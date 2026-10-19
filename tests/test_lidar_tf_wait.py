"""A wall fit gives the LiDAR's mount transform a moment when the scan arrives first."""

import math
import threading
import time
from types import SimpleNamespace

import pytest

tf2_ros = pytest.importorskip('tf2_ros')

from geometry_msgs.msg import TransformStamped  # noqa: E402

from osiris_agent import snapshots  # noqa: E402


def static_transform(parent, child, half_turn=False):
    t = TransformStamped()
    t.header.frame_id = parent
    t.child_frame_id = child
    # A quaternion's w defaults to 1.
    if half_turn:
        t.transform.rotation.z = 1.0
        t.transform.rotation.w = 0.0
    return t


class FakeAgent(snapshots.SnapshotMixin):
    def __init__(self, tf_published=True):
        self.tf_published = tf_published

    def count_publishers(self, name):
        return 1 if self.tf_published and name in ('/tf', '/tf_static') else 0


def front_angle(agent, buffer):
    holder = {'listener': SimpleNamespace(namespace='')}
    done = threading.Event()
    result = {}

    def deliver(angle):
        result['angle'], result['at'] = angle, time.monotonic()
        done.set()

    start = time.monotonic()
    agent._front_angle_when_ready(buffer, holder, 'base_scan', deliver)
    assert done.wait(2.0)
    return result['angle'], result['at'] - start


def test_a_mount_transform_arriving_after_the_scan_is_used():
    buffer = tf2_ros.Buffer()
    late = threading.Timer(0.15, lambda: buffer.set_transform_static(static_transform('base_link', 'base_scan', half_turn=True), 'test'))
    late.start()
    angle, waited = front_angle(FakeAgent(), buffer)
    assert angle == pytest.approx(math.pi)
    assert 0.1 < waited < snapshots.LIDAR_TF_WAIT_S


def test_the_wait_is_bounded():
    angle, waited = front_angle(FakeAgent(), tf2_ros.Buffer())
    assert angle is None
    assert snapshots.LIDAR_TF_WAIT_S <= waited < snapshots.LIDAR_TF_WAIT_S + 0.25


def test_a_robot_without_tf_is_not_kept_waiting():
    angle, waited = front_angle(FakeAgent(tf_published=False), tf2_ros.Buffer())
    assert angle is None and waited < 0.05


def test_a_scan_frame_without_a_path_to_base_link_is_not_waited_for():
    buffer = tf2_ros.Buffer()
    buffer.set_transform_static(static_transform('odom', 'base_scan'), 'test')
    angle, waited = front_angle(FakeAgent(), buffer)
    assert angle is None and waited < 0.05
