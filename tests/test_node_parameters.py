"""Parameter changes a node makes while the agent fetches its parameters are kept."""

import importlib.util
import sys
import types


def _installed(name):
    # A stand-in another test module installed has no spec.
    module = sys.modules.get(name)
    if module is not None:
        return module.__spec__ is not None
    return importlib.util.find_spec(name) is not None


def _install_ros_stand_ins():
    # node_parameters.py imports rcl_interfaces and rclpy.parameter at module
    # level. Where ROS is not installed, minimal stand-ins let it be imported.
    msg = types.ModuleType('rcl_interfaces.msg')
    msg.ParameterEvent = object
    srv = types.ModuleType('rcl_interfaces.srv')
    srv.ListParameters = types.SimpleNamespace(Request=lambda: types.SimpleNamespace(depth=0))
    srv.GetParameters = types.SimpleNamespace(Request=lambda: types.SimpleNamespace(names=[]))
    rcl_interfaces = types.ModuleType('rcl_interfaces')
    rcl_interfaces.msg, rcl_interfaces.srv = msg, srv
    parameter = sys.modules.get('rclpy.parameter') or types.ModuleType('rclpy.parameter')
    parameter.parameter_value_to_python = lambda value: value
    rclpy = sys.modules.get('rclpy') or types.ModuleType('rclpy')
    rclpy.parameter = parameter
    sys.modules.update({
        'rcl_interfaces': rcl_interfaces, 'rcl_interfaces.msg': msg, 'rcl_interfaces.srv': srv,
        'rclpy': rclpy, 'rclpy.parameter': parameter,
    })


if not _installed('rclpy') or not _installed('rcl_interfaces'):
    _install_ros_stand_ins()

import pytest  # noqa: E402

from osiris_agent import node_parameters  # noqa: E402
from osiris_agent.node_parameters import NodeParametersMixin  # noqa: E402

NODE = '/controller_server'


@pytest.fixture(autouse=True)
def plain_values(monkeypatch):
    # Parameter values in these tests are plain Python values already.
    monkeypatch.setattr(node_parameters, 'parameter_value_to_python', lambda value: value)


class FakeFuture:
    def __init__(self):
        self._callback = self._result = None

    def add_done_callback(self, callback):
        self._callback = callback

    def answer(self, result):
        self._result = result
        self._callback(self)

    def result(self):
        return self._result


class FakeAgent(NodeParametersMixin):
    """Parameter services whose answers the test gives by hand, in calls order."""

    def __init__(self):
        self._params_enabled = True
        self._active_nodes = {NODE}
        self._node_parameter_cache = {}
        self._pending_param_fetches = set()
        self._nodes_no_param_service = set()
        self._param_fetch_changes = {}
        self._graph_dirty = False
        self.calls = []

    def create_client(self, srv_type, name):
        agent = self

        class Client:
            def service_is_ready(self):
                return True

            def call_async(self, request):
                future = FakeFuture()
                agent.calls.append((name, future))
                return future

        return Client()

    def destroy_client(self, client):
        pass

    def get_logger(self):
        return types.SimpleNamespace(debug=lambda *a: None)

    def _trigger_graph_poll(self):
        pass

    def answer_list(self, names):
        name, future = self.calls.pop(0)
        assert name.endswith('/list_parameters')
        future.answer(types.SimpleNamespace(result=types.SimpleNamespace(names=names)))

    def answer_get(self, values):
        name, future = self.calls.pop(0)
        assert name.endswith('/get_parameters')
        future.answer(types.SimpleNamespace(values=values))

    def event(self, new=(), changed=(), deleted=()):
        p = lambda name, value=None: types.SimpleNamespace(name=name, value=value)  # noqa: E731
        self._on_parameter_event(types.SimpleNamespace(
            node=NODE,
            new_parameters=[p(*item) for item in new],
            changed_parameters=[p(*item) for item in changed],
            deleted_parameters=[p(name) for name in deleted],
        ))


def test_changes_between_the_list_and_get_answers_are_kept():
    agent = FakeAgent()
    agent._fetch_node_parameters_async(NODE)
    agent.answer_list(['speed', 'plugin', 'old'])
    agent.event(new=[('FollowPath.max_vel', 0.4)], changed=[('speed', 2.0)], deleted=['old'])
    agent.answer_get([1.0, 'dwb', 'x'])
    assert agent._node_parameter_cache[NODE] == {'speed': 2.0, 'plugin': 'dwb', 'FollowPath.max_vel': 0.4}
    assert agent._pending_param_fetches == set() and agent._param_fetch_changes == {}


def test_changes_before_the_list_answer_are_kept():
    agent = FakeAgent()
    agent._fetch_node_parameters_async(NODE)
    agent.event(new=[('late', 3)])
    agent.answer_list(['speed'])
    agent.answer_get([1.0])
    assert agent._node_parameter_cache[NODE] == {'speed': 1.0, 'late': 3}


def test_an_empty_list_answer_keeps_parameters_declared_meanwhile():
    agent = FakeAgent()
    agent._fetch_node_parameters_async(NODE)
    agent.event(new=[('late', 3)])
    agent.answer_list([])
    assert agent._node_parameter_cache[NODE] == {'late': 3}


def test_events_after_the_fetch_still_update_the_cache():
    agent = FakeAgent()
    agent._fetch_node_parameters_async(NODE)
    agent.answer_list(['speed'])
    agent.answer_get([1.0])
    agent.event(changed=[('speed', 5.0)], new=[('extra', True)])
    assert agent._node_parameter_cache[NODE] == {'speed': 5.0, 'extra': True}


def test_a_fetch_dropped_meanwhile_is_ignored():
    # The node stopped, or parameter fetching was turned off, during the fetch.
    agent = FakeAgent()
    agent._fetch_node_parameters_async(NODE)
    agent.answer_list(['speed'])
    agent._pending_param_fetches.discard(NODE)
    agent._param_fetch_changes.pop(NODE)
    agent.answer_get([1.0])
    assert NODE not in agent._node_parameter_cache


def test_events_are_ignored_while_parameter_fetching_is_off():
    agent = FakeAgent()
    agent._params_enabled = False
    agent.event(new=[('speed', 1.0)])
    assert agent._node_parameter_cache == {}


def test_a_node_fetched_in_the_poll_that_found_it_gets_changes_before_it_is_active():
    # The poll that finds a node fetches it first and marks it active last.
    agent = FakeAgent()
    agent._active_nodes = set()
    agent._fetch_node_parameters_async(NODE)
    agent.answer_list(['use_sim_time'])
    agent.answer_get([False])
    agent.event(new=[('max_velocity', [0.5, 0.0, 2.0])])
    assert agent._node_parameter_cache[NODE] == {'use_sim_time': False, 'max_velocity': [0.5, 0.0, 2.0]}


def test_events_for_a_node_that_stopped_are_ignored():
    agent = FakeAgent()
    agent._fetch_node_parameters_async(NODE)
    agent.answer_list(['speed'])
    agent.answer_get([1.0])
    # The poll that sees the node stop removes it and its cache entry.
    agent._active_nodes.discard(NODE)
    agent._node_parameter_cache.pop(NODE)
    agent.event(new=[('late', 3)])
    assert NODE not in agent._node_parameter_cache
