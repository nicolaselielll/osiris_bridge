"""The agent connects once DDS discovery has settled, so the first graph the
gateway gets is the robot's whole graph, and it never waits past a cap."""

import importlib.util
import sys
import threading
import types


def _installed(name):
    # A stand-in another test module installed has no spec.
    module = sys.modules.get(name)
    if module is not None:
        return module.__spec__ is not None
    return importlib.util.find_spec(name) is not None


if not _installed('std_msgs'):
    # graph_monitor.py imports std_msgs at module level; where ROS is not
    # installed, a minimal stand-in lets it be imported.
    msg = types.ModuleType('std_msgs.msg')
    msg.Empty = object
    std_msgs = types.ModuleType('std_msgs')
    std_msgs.msg = msg
    sys.modules.update({'std_msgs': std_msgs, 'std_msgs.msg': msg})

from osiris_agent import graph_monitor  # noqa: E402
from osiris_agent.graph_monitor import GRAPH_SETTLE_MAX_S, GRAPH_SETTLE_S, GraphMonitorMixin  # noqa: E402

START = 1000.0


class FakeTime:
    """A clock that moves only when slept on, running what falls due on the way."""

    def __init__(self, at=START):
        self.now = at
        self.due = []

    def monotonic(self):
        return self.now

    def sleep(self, seconds):
        self.now += seconds
        while self.due and self.due[0][0] <= self.now:
            self.due.pop(0)[1]()


class FakeAgent(GraphMonitorMixin):
    def __init__(self):
        self._started_at = START
        self._graph_seen = None
        self._graph_changed_at = START
        self._initial_scan_complete = threading.Event()
        self._graph_check_lock = threading.Lock()


def graph(node_count):
    return {f'/node{i}' for i in range(node_count)}, {f'/topic{i}' for i in range(node_count * 4)}


def discovery(agent, clock, scans):
    """Graph scans at the given times, each finding the given number of nodes."""
    def scan(node_count):
        def run():
            agent._note_graph(*graph(node_count))
            agent._initial_scan_complete.set()
        return run
    clock.due = sorted(((at, scan(count)) for at, count in scans), key=lambda d: d[0])


def test_it_connects_once_discovery_has_settled(monkeypatch):
    clock = FakeTime()
    monkeypatch.setattr(graph_monitor, 'time', clock)
    agent = FakeAgent()
    # As on a TurtleBot3 under load: the first scan finds 5 of its 27 nodes.
    discovery(agent, clock, [(START + 0.3, 5), (START + 1.3, 26), (START + 2.4, 27), (START + 3.6, 27)])
    agent._wait_for_graph_to_settle()
    assert agent._graph_seen == graph(27)
    assert START + 2.4 + GRAPH_SETTLE_S <= clock.now < START + 2.4 + GRAPH_SETTLE_S + 0.25


def test_a_pause_in_discovery_shorter_than_the_quiet_time_is_waited_out(monkeypatch):
    clock = FakeTime()
    monkeypatch.setattr(graph_monitor, 'time', clock)
    agent = FakeAgent()
    # As on a freshly started robot container: the first scans find 2 nodes,
    # and the other 24 show up 3.5 s later, at their next announcement.
    discovery(agent, clock, [(START + 1.3, 2), (START + 1.5, 2), (START + 4.8, 26)])
    agent._wait_for_graph_to_settle()
    assert agent._graph_seen == graph(26)
    assert START + 4.8 + GRAPH_SETTLE_S <= clock.now < START + 4.8 + GRAPH_SETTLE_S + 0.25


def test_it_never_waits_past_the_cap_while_the_graph_keeps_changing(monkeypatch):
    clock = FakeTime()
    monkeypatch.setattr(graph_monitor, 'time', clock)
    agent = FakeAgent()
    # A node comes or goes every half second.
    discovery(agent, clock, [(START + 0.5 * i, 5 + i % 2) for i in range(1, 2 * int(GRAPH_SETTLE_MAX_S) + 10)])
    agent._wait_for_graph_to_settle()
    assert START + GRAPH_SETTLE_MAX_S <= clock.now < START + GRAPH_SETTLE_MAX_S + 0.2


def test_a_reconnect_does_not_wait(monkeypatch):
    clock = FakeTime(at=START + 3600)
    monkeypatch.setattr(graph_monitor, 'time', clock)
    agent = FakeAgent()
    agent._initial_scan_complete.set()
    agent._graph_seen, agent._graph_changed_at = graph(27), START + 600
    agent._wait_for_graph_to_settle()
    assert clock.now == START + 3600


def test_only_a_different_graph_counts_as_a_change(monkeypatch):
    clock = FakeTime()
    monkeypatch.setattr(graph_monitor, 'time', clock)
    agent = FakeAgent()
    agent._note_graph(*graph(5))
    clock.now += 5
    agent._note_graph(*graph(5))
    assert agent._graph_changed_at == START
    agent._note_graph(*graph(6))
    assert agent._graph_changed_at == START + 5
