"""A graph watcher that exits is started again, so graph changes keep reaching the app."""

import importlib.util
import sys
import types


def _installed(name):
    module = sys.modules.get(name)
    if module is not None:
        return module.__spec__ is not None
    return importlib.util.find_spec(name) is not None


if not _installed('std_msgs'):
    msg = types.ModuleType('std_msgs.msg')
    msg.Empty = object
    std_msgs = types.ModuleType('std_msgs')
    std_msgs.msg = msg
    sys.modules.update({'std_msgs': std_msgs, 'std_msgs.msg': msg})

from osiris_agent import graph_monitor  # noqa: E402
from osiris_agent.graph_monitor import WATCHER_QUICK_RESTARTS, WATCHER_RESTART_WINDOW_S, GraphMonitorMixin  # noqa: E402


class Proc:
    def __init__(self, rc=None):
        self.returncode = rc
        self.stderr = None

    def poll(self):
        return self.returncode


class Node(GraphMonitorMixin):
    def __init__(self, proc):
        self._watcher_proc = proc
        self.forwarded = []
        self.polls = 0
        self.errors = []

    def get_logger(self):
        return types.SimpleNamespace(error=self.errors.append, info=lambda *_: None, debug=lambda *_: None)

    def _forward_watcher_stderr(self, proc):
        self.forwarded.append(proc)

    def _trigger_graph_poll(self):
        self.polls += 1


def patch_watcher(monkeypatch, clock, started):
    monkeypatch.setattr(graph_monitor.time, 'monotonic', lambda: clock[0])
    monkeypatch.setattr(graph_monitor, 'start_graph_watcher', lambda: started.pop(0) if started else None)


def test_a_running_watcher_is_left_alone(monkeypatch):
    patch_watcher(monkeypatch, [0.0], [Proc()])
    node = Node(Proc())
    node._check_graph_watcher()
    assert node.forwarded == [] and node.polls == 0


def test_no_watcher_from_the_start_is_not_retried(monkeypatch):
    started = [Proc()]
    patch_watcher(monkeypatch, [0.0], started)
    node = Node(None)
    node._check_graph_watcher()
    assert node._watcher_proc is None and len(started) == 1


def test_an_exited_watcher_is_started_again_and_the_graph_read_once(monkeypatch):
    new = Proc()
    patch_watcher(monkeypatch, [0.0], [new])
    node = Node(Proc(rc=-15))
    node._check_graph_watcher()
    assert node._watcher_proc is new and node.forwarded == [new] and node.polls == 1
    assert 'rc=-15' in node.errors[0]


def test_one_that_keeps_crashing_is_retried_once_a_minute(monkeypatch):
    clock = [0.0]
    patch_watcher(monkeypatch, clock, [Proc(rc=1) for _ in range(10)])
    node = Node(Proc(rc=1))
    for i in range(WATCHER_QUICK_RESTARTS + 2):
        clock[0] = i * 5.0
        node._check_graph_watcher()
    assert len(node.forwarded) == WATCHER_QUICK_RESTARTS
    clock[0] = (WATCHER_QUICK_RESTARTS - 1) * 5.0 + WATCHER_RESTART_WINDOW_S + 1
    node._check_graph_watcher()
    assert len(node.forwarded) == WATCHER_QUICK_RESTARTS + 1


def test_a_failed_restart_keeps_the_old_process_to_try_again(monkeypatch):
    patch_watcher(monkeypatch, [0.0], [])
    old = Proc(rc=1)
    node = Node(old)
    node._check_graph_watcher()
    assert node._watcher_proc is old and node.polls == 0
