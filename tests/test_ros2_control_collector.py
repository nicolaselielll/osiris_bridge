"""ros2_control state changes reach the agent even when nothing in the ROS graph changes."""

import sys
import types

import pytest

from osiris_agent import ros2_control_collector
from osiris_agent.ros2_control_collector import REQUEST_TIMEOUT_S, Ros2ControlCollector

HARDWARE = '/controller_manager/list_hardware_components'


@pytest.fixture(autouse=True)
def controller_manager_msgs(monkeypatch):
    # The collector imports the service types on first use. Stand-ins keep
    # these tests the same whether or not ros2_control is installed.
    srv = types.ModuleType('controller_manager_msgs.srv')
    srv.ListControllers = srv.ListHardwareComponents = types.SimpleNamespace(Request=object)
    package = types.ModuleType('controller_manager_msgs')
    package.srv = srv
    monkeypatch.setitem(sys.modules, 'controller_manager_msgs', package)
    monkeypatch.setitem(sys.modules, 'controller_manager_msgs.srv', srv)
    monkeypatch.setattr(Ros2ControlCollector, '_cm_msgs_available', None)


@pytest.fixture
def clock(monkeypatch):
    now = [1000.0]
    monkeypatch.setattr(ros2_control_collector, 'time', types.SimpleNamespace(time=lambda: now[0]))
    return now


class FakeFuture:
    def __init__(self):
        self._done, self._result, self._callbacks = False, None, []

    def done(self):
        return self._done

    def result(self):
        return self._result

    def add_done_callback(self, callback):
        self._callbacks.append(callback)

    def answer(self, result):
        self._done, self._result = True, result
        for callback in self._callbacks:
            callback(self)


class FakeClient:
    def __init__(self):
        self.requests, self.dropped = [], []

    def service_is_ready(self):
        return True

    def call_async(self, request):
        self.requests.append(FakeFuture())
        return self.requests[-1]

    def remove_pending_request(self, future):
        self.dropped.append(future)


class FakeNode:
    def __init__(self):
        self.clients = {}
        self.node_names = ['controller_manager']

    def get_node_names(self):
        return self.node_names

    def create_client(self, srv_type, name):
        self.clients[name] = FakeClient()
        return self.clients[name]

    def destroy_client(self, client):
        pass


def hardware_response(label):
    component = types.SimpleNamespace(
        name='MockArm', type='system', class_type='mock_components/GenericSystem',
        state=types.SimpleNamespace(label=label, id=0), command_interfaces=[], state_interfaces=[])
    return types.SimpleNamespace(component=[component])


def make_collector():
    node, events = FakeNode(), []
    collector = Ros2ControlCollector(node=node, event_callback=events.append, logger=None, poll_interval=2.0)
    return collector, node, events


def hardware_states(events):
    return [e['data'][0]['state']['label'] for e in events if e['type'] == 'hardware']


def test_a_forced_poll_reports_a_change_the_rate_limit_skips(clock):
    # The agent's poll timer forces its polls. A graph check's poll within
    # poll_interval of the last one is skipped, and nothing else polled.
    collector, node, events = make_collector()
    collector.poll(force=True)
    node.clients[HARDWARE].requests[-1].answer(hardware_response('active'))

    clock[0] += 1.0
    collector.poll()
    assert len(node.clients[HARDWARE].requests) == 1

    collector.poll(force=True)
    node.clients[HARDWARE].requests[-1].answer(hardware_response('inactive'))
    assert hardware_states(events) == ['active', 'inactive']


def test_no_second_request_while_one_is_unanswered(clock):
    collector, node, _ = make_collector()
    collector.poll(force=True)
    client = node.clients[HARDWARE]
    first = client.requests[0]

    clock[0] += REQUEST_TIMEOUT_S - 1
    collector.poll(force=True)
    assert client.requests == [first]

    clock[0] += 2
    collector.poll(force=True)
    assert client.dropped == [first]
    assert len(client.requests) == 2


def test_a_restarted_controller_manager_is_asked_again_at_once(clock):
    collector, node, _ = make_collector()
    collector.poll(force=True)
    client = node.clients[HARDWARE]

    node.node_names = []
    collector.poll(force=True)
    node.node_names = ['controller_manager']
    collector.poll(force=True)
    assert len(client.requests) == 2


def test_a_destroyed_collector_stops_polling(clock):
    collector, node, _ = make_collector()
    collector.destroy()
    collector.poll(force=True)
    assert node.clients == {}
