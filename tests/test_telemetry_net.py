"""The telemetry's network rates count only the robot's network interfaces:
loopback carries the traffic between its own processes, which never leaves it."""

from types import SimpleNamespace

import pytest

pytest.importorskip('psutil')

from osiris_agent import telemetry  # noqa: E402

MB = 1024 * 1024


def counters(**interfaces):
    return {name: SimpleNamespace(bytes_sent=sent, bytes_recv=recv) for name, (sent, recv) in interfaces.items()}


class FakeAgent(telemetry.TelemetryMixin):
    def __init__(self):
        self._last_disk_io = None
        self._last_net_io = None
        self._last_io_time = None
        self._last_battery_state = None


def test_loopback_traffic_is_left_out(monkeypatch):
    readings = [
        counters(lo=(0, 0), eth0=(0, 0)),
        # Ten seconds later: the robot's own processes moved 300 MB between
        # them over loopback, while 1 MB went out and 0.5 MB came in on eth0.
        counters(lo=(300 * MB, 300 * MB), eth0=(1 * MB, MB // 2)),
    ]
    monkeypatch.setattr(telemetry.psutil, 'net_io_counters', lambda pernic=False: readings.pop(0))
    now = [100.0]
    monkeypatch.setattr(telemetry, 'time', SimpleNamespace(time=lambda: now[0]))
    agent = FakeAgent()
    agent._get_telemetry_snapshot(include_processes=False)
    now[0] = 110.0
    assert agent._get_telemetry_snapshot(include_processes=False)['net'] == {'tx_mbps': 0.1, 'rx_mbps': 0.05}


def test_an_interface_that_appears_or_is_reset_is_not_a_burst():
    before = {'eth0': (5 * MB, 5 * MB)}
    after = {'eth0': (1 * MB, 6 * MB), 'wlan0': (900 * MB, 900 * MB)}
    # eth0's send counter went back, so it was reset: nothing sent. wlan0
    # wasn't there before: nothing yet. eth0 received 1 MB.
    assert telemetry.network_rates_mbps(before, after, 1.0) == (0.0, 1.0)


@pytest.mark.parametrize('name, loopback', [
    ('lo', True), ('lo0', True), ('eth0', False), ('wlan0', False), ('lowpan0', False),
])
def test_loopback_interfaces_are_told_apart_by_name(name, loopback):
    assert telemetry._is_loopback(name) is loopback


def test_a_few_kib_per_second_is_reported_not_rounded_to_zero():
    # 7 KiB sent in 10 s, about what a robot with nothing subscribed sends.
    tx, rx = telemetry.network_rates_mbps({'eth0': (0, 0)}, {'eth0': (7 * 1024, 0)}, 10.0)
    assert round(tx * 1024, 1) == 0.7
    assert rx == 0.0
