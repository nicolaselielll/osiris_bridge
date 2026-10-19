"""An action is something a server provides or a client uses, not a status
topic that exists only because this agent reads it."""

import importlib.util
import sys
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

from osiris_agent.graph_monitor import GraphMonitorMixin  # noqa: E402

AGENT = '/osiris_node'


def endpoint(fqn):
    ns, _, name = fqn.rpartition('/')
    return types.SimpleNamespace(node_name=name, node_namespace=ns or '/')


class FakeAgent(GraphMonitorMixin):
    """A graph given as topic -> (publishers, subscribers), polled after a
    first scan that already knew `known_actions`."""

    def __init__(self, graph, known_actions=()):
        self.graph = graph
        self.unsubscribed = []
        self._first_graph_check_done = True
        self._active_nodes = {AGENT, '/robot1/bt_navigator', '/docking_client'}
        self._active_topics = set(graph)
        self._active_services = {}
        self._active_actions = set(known_actions)
        self._service_rescan_ticks = 0
        self._pending_topic_enrichment = set()
        self._topic_relations = {}
        self._topic_counts = {}
        self._action_type_cache = {}
        self._goals_enabled = True
        self._nav2_bt_monitor_initialized = False
        self._nav2_bt_log_topic = None
        self._ros2_control = self._tf_tree = None
        self._graph_dirty = False
        self._graph_seen = None
        self._graph_changed_at = 0.0

    def get_fully_qualified_name(self):
        return AGENT

    def get_node_names_and_namespaces(self):
        return [(fqn.rpartition('/')[2], fqn.rpartition('/')[0] or '/') for fqn in self._active_nodes]

    def get_topic_names_and_types(self):
        return [(t, ['action_msgs/msg/GoalStatusArray']) for t in self.graph]

    def count_publishers(self, topic):
        return len(self.graph[topic][0])

    def count_subscribers(self, topic):
        return len(self.graph[topic][1])

    def get_publishers_info_by_topic(self, topic):
        return [endpoint(n) for n in self.graph[topic][0]]

    def get_subscriptions_info_by_topic(self, topic):
        return [endpoint(n) for n in self.graph[topic][1]]

    def get_logger(self):
        log = lambda *a, **k: None  # noqa: E731
        return types.SimpleNamespace(info=log, debug=log, warning=log)

    def _fetch_action_types(self, action, topic_type_map):
        pass

    def _subscribe_action_status(self, action):
        pass

    def _unsubscribe_action_monitoring(self, action):
        self.unsubscribed.append(action)

    def _refresh_bridge_qos(self, topics):
        pass

    def _flush_graph_snapshots(self):
        pass


GRAPH = {
    # Exists only because this agent reads it: Nav2 runs under /robot1.
    '/navigate_to_pose/_action/status': ([], [AGENT]),
    '/robot1/navigate_to_pose/_action/status': (['/robot1/bt_navigator'], [AGENT, '/robot1/bt_navigator']),
    # A client waiting for a docking server that isn't running.
    '/dock_robot/_action/status': ([], ['/docking_client']),
}


def test_a_status_topic_only_this_agent_reads_is_not_an_action():
    agent = FakeAgent(GRAPH)
    agent._check_graph_changes_locked()
    assert agent._active_actions == {'/robot1/navigate_to_pose', '/dock_robot'}


def test_a_phantom_action_seen_before_is_dropped_with_its_goal_tracking():
    agent = FakeAgent(GRAPH, known_actions={'/navigate_to_pose', '/robot1/navigate_to_pose', '/dock_robot'})
    agent._check_graph_changes_locked()
    assert agent._active_actions == {'/robot1/navigate_to_pose', '/dock_robot'}
    assert agent.unsubscribed == ['/navigate_to_pose']
