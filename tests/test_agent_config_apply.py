"""A malformed agent_config push changes nothing, except switching motion commands off."""

import copy
import importlib.util
import sys
import threading
import types

import pytest


def _installed(name):
    # A stand-in another test module installed has no spec.
    module = sys.modules.get(name)
    if module is not None:
        return module.__spec__ is not None
    return importlib.util.find_spec(name) is not None


def _install_ros_stand_ins():
    # agent_config.py imports rclpy.parameter, and qos.py rclpy.qos, at module
    # level. Where ROS is not installed, minimal stand-ins let them be imported.
    parameter = types.ModuleType('rclpy.parameter')

    class Parameter:
        Type = types.SimpleNamespace(STRING='string', DOUBLE='double', INTEGER='integer', BOOL='bool')

        def __init__(self, name, type_=None, value=None):
            self.name, self.type_, self.value = name, type_, value

    parameter.Parameter = Parameter
    qos = types.ModuleType('rclpy.qos')
    qos.QoSProfile = lambda **fields: types.SimpleNamespace(**fields)
    qos.DurabilityPolicy = types.SimpleNamespace(VOLATILE='volatile', TRANSIENT_LOCAL='transient_local')
    qos.HistoryPolicy = types.SimpleNamespace(KEEP_LAST='keep_last')
    qos.ReliabilityPolicy = types.SimpleNamespace(RELIABLE='reliable', BEST_EFFORT='best_effort')
    rclpy = sys.modules.get('rclpy') or types.ModuleType('rclpy')
    rclpy.parameter, rclpy.qos = parameter, qos
    sys.modules.update({'rclpy': rclpy, 'rclpy.parameter': parameter, 'rclpy.qos': qos})


if not _installed('rclpy'):
    _install_ros_stand_ins()

from osiris_agent import agent_config  # noqa: E402
from osiris_agent.command_policy import CommandLimits  # noqa: E402

from test_agent_config_validation import CONFIG_C  # noqa: E402

PARAMETER_DEFAULTS = {
    'bag_output_dir': '~/ros2_bags', 'graph_debounce_interval': 1.0, 'topic_data_rate_hz': 50.0,
    'topic_data_max_bytes_per_sec': 0.0, 'global_topic_data_max_bytes_per_sec': 0.0,
    'global_topic_data_max_msgs_per_sec': 0.0, 'image_jpeg_quality': 70, 'image_max_dimension': 640,
    'pointcloud_max_points': 10000, 'pointcloud_keep_intensity': False, 'pointcloud_keep_rgb': False,
    'pointcloud_default_rate_hz': 1.0,
}


class FakeCollector:
    def __init__(self, **kwargs):
        self.kwargs = kwargs

    def start(self):
        pass

    def stop(self):
        pass

    def destroy(self):
        pass


class FakeAgent(agent_config.AgentConfigMixin):
    """The node state _apply_agent_config reads and writes, starting as a
    freshly started agent with no config applied yet."""

    def __init__(self, param_overrides=None):
        self._param_overrides = param_overrides or {}
        self.parameters = dict(PARAMETER_DEFAULTS)
        self.sent = []
        self.errors = []
        self._agent_config_received = False
        self._resolved_agent_config = None
        self._telemetry_enabled_default = self._telemetry_enabled = True
        self._tf_tree_enabled_default, self._tf_tree = False, None
        self._tf_tree_poll_interval_default, self._tf_tree_poll_interval, self._tf_tree_poll_timer = 0.2, None, None
        self._ros2_control_enabled_default, self._ros2_control = False, None
        self._ros2_control_poll_interval_default, self._ros2_control_poll_interval = 2.0, None
        self._ros2_control_poll_timer = None
        self._battery_topic_default, self._battery_topic, self._battery_sub = '/battery_state', None, None
        self._battery_check_enabled_default = self._battery_check_enabled = False
        self._battery_min_percent_default = self._battery_min_percent = 30.0
        self._dock_check_enabled_default = self._dock_check_enabled = False
        self._dock_status_topic_default, self._dock_status_topic, self._dock_status_sub = '/dock_status', None, None
        self._dock_status_msg_missing_logged = False
        self._last_dock_status = self._last_dock_status_time = None
        self._command_limits_default = self._command_limits = CommandLimits()
        self._commands_enabled_default, self._commands_enabled = False, False
        self._goals_enabled_default, self._goals_enabled = True, None
        self._active_actions, self._action_status_subs, self._action_feedback_subs = set(), {}, {}
        self._params_enabled_default, self._params_enabled = True, None
        self._active_nodes, self._node_parameter_cache, self._pending_param_fetches = set(), {}, set()
        self._graph_dirty = False
        self._bt_mode_default, self._bt_mode = 'nav2', None
        self._bt_host_default, self._bt_server_port_default, self._bt_publisher_port_default = '127.0.0.1', 1667, 1668
        self._nav2_bt_monitor_initialized = False
        self._bt_collector = self._bt_collector_conn = None
        self._topic_limit_lock = threading.Lock()
        self._topic_limit_overrides = {}

    # ROS node methods
    def get_logger(self):
        log = lambda *a, **k: None  # noqa: E731
        return types.SimpleNamespace(info=log, warning=log, debug=log, error=self.errors.append)

    def create_timer(self, period, callback):
        timer = types.SimpleNamespace(period=period, callback=callback, cancelled=False)
        timer.cancel = lambda: setattr(timer, 'cancelled', True)
        return timer

    def create_subscription(self, msg_type, topic, callback, qos):
        return types.SimpleNamespace(topic=topic)

    def destroy_subscription(self, sub):
        pass

    def get_parameter(self, name):
        value = self.parameters[name]
        return types.SimpleNamespace(get_parameter_value=lambda: types.SimpleNamespace(
            string_value=value, double_value=value, integer_value=value, bool_value=value))

    def set_parameters(self, parameters):
        for p in parameters:
            self.parameters[p.name] = p.value

    def _enqueue(self, payload):
        self.sent.append(copy.deepcopy(payload))

    # The rest of the agent, not under test here
    def _init_nav2_bt_monitor(self):
        self._nav2_bt_monitor_initialized = True

    def _teardown_nav2_bt_monitor(self):
        self._nav2_bt_monitor_initialized = False

    def _subscribe_action_status(self, action):
        pass

    def _catch_up_action_feedback(self, action):
        pass

    def _unsubscribe_action_monitoring(self, action):
        pass

    def _fetch_node_parameters_async(self, fqn):
        pass

    def _poll_ros2_control(self):
        pass

    _poll_tf_tree = _on_battery_state = _on_dock_status = None
    _on_tf_tree_event = _on_ros2_control_event = _on_bt_event = None


@pytest.fixture(autouse=True)
def fake_collectors(monkeypatch):
    for name in ('TfTreeCollector', 'Ros2ControlCollector', 'BTCollector'):
        monkeypatch.setattr(agent_config, name, FakeCollector)


SETTINGS = (
    '_agent_config_received', '_telemetry_enabled', '_tf_tree_poll_interval', '_ros2_control_poll_interval',
    '_battery_topic', '_battery_check_enabled', '_battery_min_percent', '_dock_check_enabled',
    '_dock_status_topic', '_command_limits', '_commands_enabled', '_goals_enabled', '_params_enabled',
    '_bt_mode', '_bt_collector_conn', '_nav2_bt_monitor_initialized', '_topic_limit_overrides',
)


def settings(agent):
    state = {name: copy.deepcopy(getattr(agent, name)) for name in SETTINGS}
    state.update(tf_tree_on=agent._tf_tree is not None, ros2_control_on=agent._ros2_control is not None,
                 parameters=dict(agent.parameters))
    return state


# Changes to fields handled before the bad one, plus fields the push leaves
# out. None of them may change or be reset to its default.
BAD_PUSH = {
    **{k: v for k, v in CONFIG_C.items() if k not in ('tf_tree_enabled', 'ros2_control_enabled')},
    'telemetry_enabled': False, 'tf_tree_poll_interval': 3.0, 'battery_check_enabled': True,
    'image_jpeg_quality': 20,
    'topic_limits': {'/fake/string': {'rate_hz': 'fast'}},
}


def test_a_malformed_push_leaves_every_setting_in_place():
    agent = FakeAgent()
    agent._apply_agent_config({**CONFIG_C, 'commands_enabled': False})
    before, sent_before = settings(agent), len(agent.sent)

    agent._apply_agent_config(BAD_PUSH)

    assert settings(agent) == before
    assert len(agent.sent) == sent_before
    assert len(agent.errors) == 1
    assert "topic_limits[/fake/string].rate_hz='fast'" in agent.errors[0]


def test_a_malformed_push_switches_commands_off_and_says_so():
    agent = FakeAgent()
    agent._apply_agent_config(CONFIG_C)
    assert agent._commands_enabled is True
    before = settings(agent)

    agent._apply_agent_config(BAD_PUSH)

    after = settings(agent)
    assert after.pop('_commands_enabled') is False
    before.pop('_commands_enabled')
    assert after == before
    resolved = agent.sent[-1]
    assert resolved['type'] == 'resolved_agent_config'
    assert resolved['config']['commands_enabled'] is False
    assert {k: v for k, v in resolved['config'].items() if k != 'commands_enabled'} == \
        {k: v for k, v in agent.sent[-2]['config'].items() if k != 'commands_enabled'}


def test_ros2_control_is_polled_on_its_own_timer():
    # A hardware component changing state, or a controller switched twice
    # within one interval, changes nothing in the ROS graph, so the graph
    # check's poll alone left the reported state stale.
    agent = FakeAgent()
    agent._apply_agent_config({'ros2_control_enabled': True, 'ros2_control_poll_interval': 3.0})
    first = agent._ros2_control_poll_timer
    assert (first.period, first.callback) == (3.0, agent._poll_ros2_control)

    agent._apply_agent_config({'ros2_control_enabled': True, 'ros2_control_poll_interval': 0.5})
    second = agent._ros2_control_poll_timer
    assert first.cancelled
    assert (second.period, second.cancelled) == (0.5, False)

    agent._apply_agent_config({'ros2_control_enabled': False})
    assert second.cancelled
    assert agent._ros2_control_poll_timer is None


def test_a_params_file_poll_interval_of_zero_gets_no_ros2_control_timer():
    # A params file isn't validated, and a timer with period 0 would spin.
    agent = FakeAgent(param_overrides={'ros2_control_poll_interval': 0.0})
    agent._ros2_control_enabled_default, agent._ros2_control_poll_interval_default = True, 0.0
    agent._apply_agent_config({})
    assert agent._ros2_control is not None
    assert agent._ros2_control_poll_timer is None


def test_a_malformed_first_push_leaves_the_fallback_to_apply_defaults():
    agent = FakeAgent()
    agent._apply_agent_config({'battery_min_percent': 'not-a-number'})
    assert agent._agent_config_received is False
    assert agent.sent == []


def test_a_valid_push_after_a_rejected_one_applies():
    agent = FakeAgent()
    agent._apply_agent_config(CONFIG_C)
    agent._apply_agent_config(BAD_PUSH)
    agent._apply_agent_config(CONFIG_C)
    assert agent._commands_enabled is True
    assert agent._tf_tree_poll_interval == 1.0
    assert agent.sent[-1]['config']['commands_enabled'] is True


def test_a_params_file_run_ignores_the_cloud_push_without_checking_it():
    agent = FakeAgent(param_overrides={'telemetry_enabled': True})
    agent._apply_agent_config({'battery_min_percent': 'not-a-number'})
    assert agent.errors == []
    assert agent._agent_config_received is True
    assert agent._battery_min_percent == 30.0


def logging_agent():
    agent = FakeAgent()
    infos, warnings = [], []
    agent.get_logger = lambda: types.SimpleNamespace(
        info=infos.append, warning=warnings.append, debug=lambda *a, **k: None, error=agent.errors.append)
    return agent, infos, warnings


def dock_lines(lines):
    return [line for line in lines if 'Dock status' in line]


def install_dock_status_msg(monkeypatch):
    msg = types.ModuleType('opennav_docking_msgs.msg')
    msg.DockStatus = object
    package = types.ModuleType('opennav_docking_msgs')
    package.msg = msg
    monkeypatch.setitem(sys.modules, 'opennav_docking_msgs', package)
    monkeypatch.setitem(sys.modules, 'opennav_docking_msgs.msg', msg)


def test_a_robot_without_docking_is_told_once_at_info(monkeypatch):
    # The subscription stays unset, so every apply tries the import again.
    monkeypatch.setitem(sys.modules, 'opennav_docking_msgs.msg', None)
    agent, infos, warnings = logging_agent()

    for config in (CONFIG_C, CONFIG_C, {}):
        agent._apply_agent_config(config)

    assert agent._dock_status_sub is None
    assert dock_lines(infos) == [
        "Dock status is off: this robot's ROS install has no opennav_docking_msgs DockStatus message"]
    assert dock_lines(warnings) == []


def test_other_dock_subscription_failures_still_warn(monkeypatch):
    install_dock_status_msg(monkeypatch)
    agent, infos, warnings = logging_agent()

    def create_subscription(msg_type, topic, callback, qos):
        if topic == '/dock_status':
            raise ValueError('invalid topic name')
        return types.SimpleNamespace(topic=topic)

    agent.create_subscription = create_subscription
    agent._apply_agent_config(CONFIG_C)

    assert dock_lines(infos) == []
    assert dock_lines(warnings) == ['Dock status monitoring unavailable: invalid topic name']


def test_dock_status_is_picked_up_once_its_message_is_installed(monkeypatch):
    monkeypatch.setitem(sys.modules, 'opennav_docking_msgs.msg', None)
    agent, infos, warnings = logging_agent()
    agent._apply_agent_config(CONFIG_C)

    install_dock_status_msg(monkeypatch)
    agent._apply_agent_config(CONFIG_C)

    assert agent._dock_status_sub.topic == '/dock_status'
    assert dock_lines(infos)[-1] == 'Dock status subscription active on /dock_status'
    assert dock_lines(warnings) == []


def test_a_cloud_bag_folder_changes_nothing():
    agent = FakeAgent()
    agent._apply_agent_config({**copy.deepcopy(CONFIG_C), 'bag_output_dir': '/home/robot'})
    assert agent.parameters['bag_output_dir'] == '~/ros2_bags'
    assert agent.parameters['graph_debounce_interval'] == CONFIG_C['graph_debounce_interval']
    [resolved] = [m for m in agent.sent if m['type'] == 'resolved_agent_config']
    assert resolved['config']['bag_output_dir'] == '~/ros2_bags'


def test_a_bag_folder_set_on_the_robot_keeps_the_cloud_configuration():
    # osiris --ros-args -p bag_output_dir:=/data/bags, or a params file with
    # only that: the rest still comes from the cloud configuration.
    assert agent_config.cloud_config_overrides({'bag_output_dir'}) == set()
    assert agent_config.cloud_config_overrides({'bag_output_dir', 'commands_enabled'}) == {'commands_enabled'}
    agent = FakeAgent(param_overrides=agent_config.cloud_config_overrides({'bag_output_dir'}))
    agent.parameters['bag_output_dir'] = '/data/bags'
    agent._apply_agent_config(copy.deepcopy(CONFIG_C))
    assert agent.parameters['graph_debounce_interval'] == CONFIG_C['graph_debounce_interval']
    assert agent.parameters['bag_output_dir'] == '/data/bags'
    [resolved] = [m for m in agent.sent if m['type'] == 'resolved_agent_config']
    assert resolved['yaml_override'] is False
    assert resolved['config']['bag_output_dir'] == '/data/bags'

