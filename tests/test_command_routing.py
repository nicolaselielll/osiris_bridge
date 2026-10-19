"""Subscription QoS matching, drive goal signs, and which action servers and
behavior tree log the agent uses on a real graph (root or namespaced Nav2)."""

from types import SimpleNamespace

import pytest

# Needs ROS: commands.py imports rclpy.action. A stand-in rclpy that another
# test module installed has no rclpy.action, so this skips there too.
pytest.importorskip('rclpy.action')

from rclpy.qos import DurabilityPolicy, ReliabilityPolicy  # noqa: E402

from osiris_agent.commands import CommandMixin  # noqa: E402
from osiris_agent.nav2_bt_monitor import Nav2BtMonitorMixin  # noqa: E402
from osiris_agent.qos import matching_qos  # noqa: E402


def publisher(reliability, durability):
    return SimpleNamespace(qos_profile=SimpleNamespace(reliability=reliability, durability=durability))


class FakeGraphNode:
    def __init__(self, publishers=None):
        self.publishers = publishers or {}

    def get_publishers_info_by_topic(self, topic):
        return self.publishers.get(topic, [])


RELIABLE, BEST_EFFORT = ReliabilityPolicy.RELIABLE, ReliabilityPolicy.BEST_EFFORT
VOLATILE, LATCHED = DurabilityPolicy.VOLATILE, DurabilityPolicy.TRANSIENT_LOCAL


@pytest.mark.parametrize('publishers, reliability, durability', [
    ([], RELIABLE, VOLATILE),                                                    # nothing published yet
    ([publisher(RELIABLE, VOLATILE)], RELIABLE, VOLATILE),
    ([publisher(BEST_EFFORT, VOLATILE)], BEST_EFFORT, VOLATILE),                 # most LiDARs, simulators
    ([publisher(RELIABLE, VOLATILE), publisher(BEST_EFFORT, VOLATILE)], BEST_EFFORT, VOLATILE),
    ([publisher(RELIABLE, LATCHED)], RELIABLE, LATCHED),                         # /map, /amcl_pose
    ([publisher(RELIABLE, LATCHED), publisher(RELIABLE, VOLATILE)], RELIABLE, VOLATILE),
])
def test_subscription_qos_matches_the_publishers(publishers, reliability, durability):
    qos = matching_qos(FakeGraphNode({'/t': publishers}), '/t', depth=5)
    assert (qos.reliability, qos.durability, qos.depth) == (reliability, durability, 5)


class FakeAgent(CommandMixin, Nav2BtMonitorMixin):
    def __init__(self, actions=(), namespace='/', goal_states=None, types=None, topics=None):
        self._active_actions = set(actions)
        self._action_type_cache = types or {}
        self._action_goal_states = goal_states or {}
        self._namespace = namespace
        self._topics = topics or {}

    def get_namespace(self):
        return self._namespace

    def get_topic_names_and_types(self):
        return [(t, ['nav2_msgs/msg/BehaviorTreeLog']) for t in self._topics]

    def count_publishers(self, topic):
        return self._topics.get(topic, 0)

    def _apply_shared_goal_fields(self, goal, command, time_allowance_s, disable_collision_checks):
        pass


@pytest.mark.parametrize('distance, speed, sent_speed', [(0.5, 0.15, 0.15), (-0.3, 0.1, -0.1), (-0.3, -0.1, -0.1)])
def test_a_backward_drive_sends_a_negative_speed(distance, speed, sent_speed):
    from nav2_msgs.action import DriveOnHeading
    goal = FakeAgent()._build_command_goal('drive', {'distance_m': distance, 'speed_mps': speed}, DriveOnHeading)
    assert goal.target.x == pytest.approx(distance)
    assert goal.speed == pytest.approx(sent_speed)


@pytest.mark.parametrize('actions, namespace, expected', [
    (['/spin', '/drive_on_heading'], '/', '/spin'),                       # stock Nav2 at the root
    (['/robot1/spin'], '/', '/robot1/spin'),                              # Nav2 under a namespace, agent at the root
    (['/robot1/spin', '/robot2/spin'], '/robot2', '/robot2/spin'),       # the agent's own namespace wins
    (['/robot1/spin', '/robot2/spin'], '/', None),                       # ambiguous: not guessed
    ([], '/', '/spin'),                                                   # no server: the usual not-available reply
])
def test_spin_goes_to_the_right_action_server(actions, namespace, expected):
    assert FakeAgent(actions=actions, namespace=namespace)._resolve_command_action_name('spin') == expected


def test_an_action_with_the_right_name_but_another_type_is_not_used():
    agent = FakeAgent(actions=['/spin', '/robot1/spin'],
                      types={'/spin': {'goal_type': 'my_msgs/action/Spin_Goal'},
                             '/robot1/spin': {'goal_type': 'nav2_msgs/action/Spin_Goal'}})
    assert agent._resolve_command_action_name('spin') == '/robot1/spin'


def test_a_running_navigation_blocks_drive_and_spin():
    active = {'/robot1/navigate_to_pose': {'a1': 2}}
    assert FakeAgent(actions=['/robot1/navigate_to_pose'], goal_states=active)._active_motion_action() == '/robot1/navigate_to_pose'
    finished = {'/navigate_to_pose': {'a1': 4}, '/compute_path_to_pose': {'b1': 2}}
    assert FakeAgent(actions=['/navigate_to_pose', '/compute_path_to_pose'], goal_states=finished)._active_motion_action() is None
    gone = {'/navigate_to_pose': {'a1': 2}}
    assert FakeAgent(actions=[], goal_states=gone)._active_motion_action() is None


def rejection(agent, command='drive'):
    import threading
    from osiris_agent.command_policy import CommandLimits
    agent._commands_enabled = True
    agent._goals_enabled = True
    agent._command_limits = CommandLimits()
    agent._active_command_lock = threading.Lock()
    agent._active_command_pending = False
    agent._active_command_goal_handle = getattr(agent, '_active_command_goal_handle', None)
    sent = []
    agent._enqueue = sent.append
    agent._handle_send_command_request({'request_id': 'r1', 'command': command, 'params': {'distance_m': 0.2}})
    return sent[0]['reason'] if sent else None


def test_the_agents_own_running_command_is_reported_as_already_in_progress():
    # Its own drive goal is in the action status as executing, like any other.
    agent = FakeAgent(actions=['/drive_on_heading'], goal_states={'/drive_on_heading': {'g1': 2}})
    agent._active_command_goal_handle = object()
    assert rejection(agent) == 'command_already_in_progress'


def test_another_goal_moving_the_robot_refuses_and_frees_the_command_slot():
    agent = FakeAgent(actions=['/navigate_to_pose'], goal_states={'/navigate_to_pose': {'n1': 2}})
    assert rejection(agent) == 'motion_in_progress:/navigate_to_pose'
    assert agent._active_command_pending is False


@pytest.mark.parametrize('topics, namespace, expected', [
    ({'/behavior_tree_log': 1}, '/', ''),
    ({'/behavior_tree_log': 0, '/robot1/behavior_tree_log': 1}, '/', '/robot1'),   # root topic only has our subscription
    ({'/robot1/behavior_tree_log': 1, '/robot2/behavior_tree_log': 1}, '/robot2', '/robot2'),
    ({'/robot1/behavior_tree_log': 1, '/robot2/behavior_tree_log': 1}, '/', ''),
    ({}, '/robot3', '/robot3'),
])
def test_the_behavior_tree_of_the_right_nav2_is_followed(topics, namespace, expected):
    assert FakeAgent(topics=topics, namespace=namespace)._resolve_nav2_namespace() == expected


def test_a_limit_refusal_is_sent_with_what_it_means():
    import math
    from osiris_agent.command_policy import CommandLimits
    agent = FakeAgent(actions=['/spin'])
    agent._commands_enabled = True
    agent._goals_enabled = True
    agent._command_limits = CommandLimits()
    sent = []
    agent._enqueue = sent.append
    agent._handle_send_command_request({'request_id': 'r1', 'command': 'spin', 'params': {'target_yaw_rad': math.radians(400)}})
    [refusal] = sent
    assert refusal['reason'] == 'limit_exceeded:target_yaw_rad'
    assert refusal['message'] == "angle_deg 400 is over this robot's limit of 360 (command_max_angle_deg)"
