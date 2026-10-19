"""The goals an action server still lists as finished when the agent
subscribes, after a restart for example, are sent as replayed, not as goals
that just ended."""

from types import SimpleNamespace

import pytest

# Needs ROS: action_monitor.py imports rclpy.qos's action status profile and
# rosidl_runtime_py. A stand-in rclpy that another test module installed has
# no such profile, so this skips there too.
try:
    from osiris_agent.action_monitor import ActionMonitorMixin
except ImportError:
    pytest.skip('needs ROS 2', allow_module_level=True)

OLD, RUNNING = b'\x01' * 16, b'\x02' * 16
SUCCEEDED, EXECUTING, ABORTED = 4, 2, 6


def status_array(*goals):
    return SimpleNamespace(status_list=[
        SimpleNamespace(goal_info=SimpleNamespace(goal_id=SimpleNamespace(uuid=uuid)), status=status)
        for uuid, status in goals
    ])


class FakeAgent(ActionMonitorMixin):
    def __init__(self):
        self.sent = []
        self._action_status_subs = {}
        self._action_feedback_subs = {}
        self._action_goal_states = {}
        self._action_feedback_throttle = {}
        self._action_status_unread = set()

    def _enqueue(self, payload):
        self.sent.append(payload)

    def create_subscription(self, *args, **kwargs):
        return object()

    def destroy_subscription(self, sub):
        pass

    def get_logger(self):
        return SimpleNamespace(info=lambda *a: None, warning=lambda *a: None)


def events(agent):
    out = [(e['goal_id'][:2], e['status'], e.get('replayed', False)) for e in agent.sent]
    agent.sent.clear()
    return out


def test_finished_goals_in_a_new_subscriptions_first_message_are_replayed():
    agent = FakeAgent()
    agent._action_status_unread.add('/spin')  # as _subscribe_action_status leaves it
    # The kept status: a spin that ended long ago, and one running now.
    agent._on_action_status(status_array((OLD, SUCCEEDED), (RUNNING, EXECUTING)), '/spin')
    assert events(agent) == [('01', SUCCEEDED, True), ('02', EXECUTING, False)]
    # Then live: the running one ends.
    agent._on_action_status(status_array((OLD, SUCCEEDED), (RUNNING, SUCCEEDED)), '/spin')
    assert events(agent) == [('02', SUCCEEDED, False)]


def test_a_goal_that_ends_while_subscribed_is_not_replayed():
    agent = FakeAgent()
    agent._action_status_unread.add('/navigate_to_pose')
    agent._on_action_status(status_array(), '/navigate_to_pose')
    agent._on_action_status(status_array((RUNNING, EXECUTING)), '/navigate_to_pose')
    agent._on_action_status(status_array((RUNNING, ABORTED)), '/navigate_to_pose')
    assert events(agent) == [('02', EXECUTING, False), ('02', ABORTED, False)]


def test_subscribing_again_replays_again():
    pytest.importorskip('action_msgs.msg')
    agent = FakeAgent()
    agent._subscribe_action_status('/spin')
    agent._on_action_status(status_array((OLD, SUCCEEDED)), '/spin')
    agent._unsubscribe_action_monitoring('/spin')
    agent._subscribe_action_status('/spin')
    agent._on_action_status(status_array((OLD, SUCCEEDED)), '/spin')
    assert events(agent) == [('01', SUCCEEDED, True), ('01', SUCCEEDED, True)]
