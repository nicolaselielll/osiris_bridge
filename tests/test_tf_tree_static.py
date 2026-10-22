"""Static transforms and the root frame are never stale in the TF tree the agent reports."""

import time
from types import SimpleNamespace

import pytest

# PyYAML comes with ROS; without it this skips.
yaml = pytest.importorskip('yaml')

from osiris_agent.tf_tree_collector import TfTreeCollector  # noqa: E402


def transform(parent, child, stamp_sec):
    sec = int(stamp_sec)
    return SimpleNamespace(
        header=SimpleNamespace(stamp=SimpleNamespace(sec=sec, nanosec=int((stamp_sec - sec) * 1e9)), frame_id=parent),
        transform=SimpleNamespace(translation=SimpleNamespace(x=0.0, y=0.0, z=0.0),
                                  rotation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0)),
    )


class FakeBuffer:
    """map -> odom -> base_footprint (dynamic) -> base_link -> base_scan (static)."""

    def __init__(self, now, odom_age=0.1):
        self.parents = {'odom': 'map', 'base_footprint': 'odom', 'base_link': 'base_footprint', 'base_scan': 'base_link'}
        self.stamps = {'odom': now - odom_age, 'base_footprint': now - 0.05, 'base_link': 0.0, 'base_scan': 0.0}

    def _getFrameStrings(self):
        return ['map', *self.parents]

    def all_frames_as_yaml(self):
        return yaml.safe_dump({child: {'parent': parent} for child, parent in self.parents.items()})

    def lookup_transform(self, parent, child, _time):
        return transform(parent, child, self.stamps[child])


def tree(buffer):
    collector = TfTreeCollector(node=None, event_callback=lambda _e: None, logger=SimpleNamespace(debug=lambda *_: None))
    collector._tf_buffer = buffer
    return collector._build_frame_tree()


def test_static_and_root_frames_are_not_stale():
    frames = tree(FakeBuffer(time.time()))
    assert frames['base_link']['is_static'] and not frames['base_link']['is_stale']
    assert frames['base_link']['age_seconds'] is None
    assert frames['base_scan']['is_static'] and not frames['base_scan']['is_stale']
    assert frames['map']['parent'] is None and not frames['map']['is_stale']
    assert not frames['odom']['is_static'] and not frames['odom']['is_stale']


def test_a_dynamic_transform_that_stopped_is_still_stale():
    frames = tree(FakeBuffer(time.time(), odom_age=30.0))
    assert frames['odom']['is_stale'] and not frames['odom']['is_static']


class SimBuffer(FakeBuffer):
    """The same tree stamped in simulation time: seconds since the simulation started."""

    def __init__(self, sim_now, odom_age=0.05, footprint_age=0.02):
        super().__init__(sim_now)
        self.stamps.update({'odom': sim_now - odom_age, 'base_footprint': sim_now - footprint_age})


def test_simulation_time_ages_count_from_the_robots_own_clock():
    frames = tree(SimBuffer(847.0))
    assert not frames['odom']['is_stale'] and not frames['base_footprint']['is_stale']
    assert frames['odom']['age_seconds'] == pytest.approx(0.03, abs=1e-3)
    assert frames['base_footprint']['age_seconds'] == pytest.approx(0.0, abs=1e-3)


def test_under_simulation_time_a_transform_that_stopped_is_still_stale():
    frames = tree(SimBuffer(847.0, odom_age=12.0))
    # Measured from the newest transform, base_footprint, 0.02 s behind the simulation.
    assert frames['odom']['is_stale'] and frames['odom']['age_seconds'] == pytest.approx(11.98, abs=1e-3)
    assert not frames['base_footprint']['is_stale']


def test_reference_time_is_the_wall_clock_unless_every_stamp_predates_2001():
    from osiris_agent.tf_tree_collector import reference_time
    wall = 1_792_530_000.0
    assert reference_time([wall - 0.1, wall - 30.0], wall) == wall
    assert reference_time([], wall) == wall
    assert reference_time([846.9, 847.0], wall) == 847.0
    # A robot whose own clock was never set (both near 1970) keeps comparing as before.
    assert reference_time([12.0], 15.0) == 15.0


def test_with_the_simulation_clock_amcls_ahead_stamp_doesnt_make_other_frames_stale():
    # AMCL stamps map -> odom transform_tolerance (1 s) ahead of the simulation.
    buffer = SimBuffer(847.0, odom_age=-1.0, footprint_age=0.02)
    collector = TfTreeCollector(node=None, event_callback=lambda _e: None, logger=SimpleNamespace(debug=lambda *_: None))
    collector._tf_buffer = buffer
    collector._sim_clock = 847.0
    frames = collector._build_frame_tree()
    assert frames['base_footprint']['age_seconds'] == pytest.approx(0.02, abs=1e-3)
    assert not frames['base_footprint']['is_stale'] and not frames['odom']['is_stale']


def test_reference_time_prefers_the_simulation_clock():
    from osiris_agent.tf_tree_collector import reference_time
    wall = 1_792_530_000.0
    assert reference_time([846.0, 848.0], wall, sim_clock=847.0) == 847.0
    # The wall-clock case ignores any simulation clock.
    assert reference_time([wall - 0.1], wall, sim_clock=847.0) == wall
