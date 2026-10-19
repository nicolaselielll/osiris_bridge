"""Graph triggers are debounced into polls without ever postponing them for good."""

import importlib.util
import sys
import types

import pytest


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
from osiris_agent.graph_monitor import GraphMonitorMixin  # noqa: E402

INTERVAL = 1.0


class Clock:
    """Fake time and timers: a timer runs its function once the clock passes its due time."""

    def __init__(self):
        self.now = 0.0
        self.timers = []

    def timer_class(self):
        clock = self

        class FakeTimer:
            def __init__(self, interval, function):
                self.due, self.function, self.cancelled, self.daemon = clock.now + interval, function, False, False
                clock.timers.append(self)

            def start(self):
                pass

            def cancel(self):
                self.cancelled = True

        return FakeTimer

    def advance(self, to):
        while True:
            due = [t for t in self.timers if not t.cancelled and t.due <= to and not getattr(t, 'fired', False)]
            if not due:
                break
            timer = min(due, key=lambda t: t.due)
            self.now = timer.due
            timer.fired = True
            timer.function()
        self.now = to


class FakeAgent(GraphMonitorMixin):
    def __init__(self, clock):
        self.clock = clock
        self.polls = []
        self._graph_debounce_timer = None

    def get_parameter(self, name):
        assert name == 'graph_debounce_interval'
        return types.SimpleNamespace(get_parameter_value=lambda: types.SimpleNamespace(double_value=INTERVAL))

    def get_logger(self):
        return types.SimpleNamespace(debug=lambda *a: None)

    def _check_graph_changes(self):
        self.polls.append(round(self.clock.now, 2))


@pytest.fixture
def setup(monkeypatch):
    clock = Clock()
    monkeypatch.setattr(graph_monitor.threading, 'Timer', clock.timer_class())
    return clock, FakeAgent(clock)


def test_a_steady_stream_of_triggers_still_polls_once_per_interval(setup):
    clock, agent = setup
    # A parameter changing every 0.3 s for 5 s.
    for step in range(17):
        clock.advance(step * 0.3)
        agent._trigger_graph_poll()
    clock.advance(6.0)
    assert len(agent.polls) >= 5
    assert all(b - a <= INTERVAL + 0.3 for a, b in zip([0.0] + agent.polls, agent.polls))


def test_a_burst_becomes_one_poll_one_interval_after_it_starts(setup):
    clock, agent = setup
    for at in (0.0, 0.1, 0.2, 0.5):
        clock.advance(at)
        agent._trigger_graph_poll()
    clock.advance(3.0)
    assert agent.polls == [1.0]


def test_a_single_trigger_polls_once_and_no_thread_is_started_per_trigger(setup):
    clock, agent = setup
    agent._trigger_graph_poll()
    clock.advance(5.0)
    assert agent.polls == [1.0]
    for step in range(10):
        clock.advance(5.0 + step * 0.05)
        agent._trigger_graph_poll()
    clock.advance(10.0)
    assert len(clock.timers) == 2


def test_a_trigger_during_a_poll_gets_a_poll_of_its_own(setup):
    clock, agent = setup
    real_check = agent._check_graph_changes

    def poll_that_sees_a_change():
        real_check()
        if len(agent.polls) == 1:
            agent._trigger_graph_poll()

    agent._check_graph_changes = poll_that_sees_a_change
    agent._trigger_graph_poll()
    clock.advance(5.0)
    assert agent.polls == [1.0, 2.0]
