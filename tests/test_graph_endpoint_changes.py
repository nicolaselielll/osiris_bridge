"""A publisher or subscription a running node adds or drops shows up in the graph."""

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

ODOM = '/odom'


def endpoint(node):
    return types.SimpleNamespace(node_name=node, node_namespace='/')


class FakeAgent(GraphMonitorMixin):
    """A graph where /base publishes /odom, as the first poll saw it. Tests
    change `publishers` and `subscribers` the way running nodes would."""

    def __init__(self):
        self.publishers, self.subscribers = ['base'], []
        self.endpoint_reads = 0
        self.flushed_dirty = []
        self._first_graph_check_done = True
        self._active_nodes = {'/base', '/controller_server'}
        self._active_topics = {ODOM}
        self._active_services = {}
        self._active_actions = set()
        self._service_rescan_ticks = 0
        self._pending_topic_enrichment = set()
        self._topic_relations = {ODOM: {
            'publishers': {'/base'}, 'subscribers': set(),
            'publisher_infos': [endpoint('base')], 'subscriber_infos': [],
            'type': 'nav_msgs/msg/Odometry',
        }}
        self._topic_counts = {ODOM: (1, 0)}
        self._action_type_cache = {}
        self._nav2_bt_monitor_initialized = False
        self._nav2_bt_log_topic = None
        self._ros2_control = self._tf_tree = None
        self._graph_dirty = False
        self._graph_seen = None
        self._graph_changed_at = 0.0

    def get_node_names_and_namespaces(self):
        return [('base', '/'), ('controller_server', '/')]

    def get_topic_names_and_types(self):
        return [(ODOM, ['nav_msgs/msg/Odometry'])]

    def count_publishers(self, topic):
        return len(self.publishers)

    def count_subscribers(self, topic):
        return len(self.subscribers)

    def get_publishers_info_by_topic(self, topic):
        self.endpoint_reads += 1
        return [endpoint(n) for n in self.publishers]

    def get_subscriptions_info_by_topic(self, topic):
        return [endpoint(n) for n in self.subscribers]

    def get_logger(self):
        log = lambda *a, **k: None  # noqa: E731
        return types.SimpleNamespace(info=log, debug=log, warning=log)

    def _refresh_bridge_qos(self, topics):
        pass

    def _flush_graph_snapshots(self):
        self.flushed_dirty.append(self._graph_dirty)
        self._graph_dirty = False


def test_a_subscription_added_by_a_running_node_is_picked_up():
    agent = FakeAgent()
    agent.subscribers = ['controller_server']
    agent._check_graph_changes_locked()
    assert agent._topic_relations[ODOM]['subscribers'] == {'/controller_server'}
    assert agent.flushed_dirty == [True]


def test_a_dropped_subscription_is_picked_up():
    agent = FakeAgent()
    agent.subscribers = ['controller_server']
    agent._check_graph_changes_locked()
    agent.subscribers = []
    agent._check_graph_changes_locked()
    assert agent._topic_relations[ODOM]['subscribers'] == set()
    assert agent.flushed_dirty == [True, True]


def test_a_second_publisher_from_a_node_already_publishing_is_picked_up():
    agent = FakeAgent()
    agent.publishers = ['base', 'base']
    agent._check_graph_changes_locked()
    assert len(agent._topic_relations[ODOM]['publisher_infos']) == 2
    assert agent.flushed_dirty == [True]


def test_an_unchanged_topic_is_not_read_again():
    agent = FakeAgent()
    agent._check_graph_changes_locked()
    assert agent.endpoint_reads == 0
    assert agent.flushed_dirty == [False]
