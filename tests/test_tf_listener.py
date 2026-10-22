"""TF is read where the robot publishes it, including under a Nav2 namespace."""

from types import SimpleNamespace

import pytest

tf2_ros = pytest.importorskip('tf2_ros')

from osiris_agent.tf_listener import TfListener, resolve_tf_namespace, tf_qos  # noqa: E402
from osiris_agent.tf_tree_collector import TfTreeCollector  # noqa: E402

TF = ['tf2_msgs/msg/TFMessage']


class FakeNode:
    def __init__(self, published, topics=None, namespace='/'):
        # Every TF topic named in published exists; topics adds ones with no publisher.
        self.topics = {name: TF for name in published}
        self.topics.update(topics or {})
        self.published = set(published)
        self.namespace = namespace
        self.subscriptions = []
        self.destroyed = []

    def get_topic_names_and_types(self):
        return list(self.topics.items())

    def count_publishers(self, name):
        return 1 if name in self.published else 0

    def get_namespace(self):
        return self.namespace

    def create_subscription(self, msg_type, topic, callback, qos, callback_group=None):
        sub = SimpleNamespace(topic=topic, callback=callback, qos=qos)
        self.subscriptions.append(sub)
        return sub

    def destroy_subscription(self, sub):
        self.destroyed.append(sub)


def test_root_tf_is_read_when_published_there():
    node = FakeNode(['/tf', '/tf_static', '/robot1/tf'])
    assert resolve_tf_namespace(node) == ''


def test_a_namespaced_nav2s_tf_is_found():
    # The agent's own subscription makes /tf exist with no publisher.
    node = FakeNode(['/robot1/tf', '/robot1/tf_static'], topics={'/tf': TF, '/tf_static': TF})
    assert resolve_tf_namespace(node) == '/robot1'


def test_static_tf_alone_counts():
    assert resolve_tf_namespace(FakeNode(['/robot1/tf_static'])) == '/robot1'


def test_several_robots_resolve_to_the_agents_own_namespace_or_the_root():
    published = ['/robot1/tf', '/robot2/tf']
    assert resolve_tf_namespace(FakeNode(published, namespace='/robot2')) == '/robot2'
    assert resolve_tf_namespace(FakeNode(published)) == ''


def test_a_topic_named_tf_of_another_type_is_ignored():
    node = FakeNode([], topics={'/robot1/tf': ['std_msgs/msg/String']})
    node.published.add('/robot1/tf')
    assert resolve_tf_namespace(node) == ''


def test_listener_subscribes_under_the_namespace_and_feeds_the_buffer():
    from geometry_msgs.msg import TransformStamped
    from rclpy.time import Time
    from tf2_msgs.msg import TFMessage

    node = FakeNode(['/robot1/tf'])
    buffer = tf2_ros.Buffer()
    listener = TfListener(buffer, node, '/robot1')
    assert [(s.topic, s.qos) for s in node.subscriptions] == [('/robot1/tf', tf_qos()), ('/robot1/tf_static', tf_qos(static=True))]

    scan_mount = TransformStamped()
    scan_mount.header.frame_id = 'base_link'
    scan_mount.child_frame_id = 'base_scan'
    scan_mount.transform.translation.x = 0.1
    scan_mount.transform.rotation.w = 1.0
    node.subscriptions[1].callback(TFMessage(transforms=[scan_mount]))
    assert buffer.lookup_transform('base_link', 'base_scan', Time()).transform.translation.x == 0.1

    listener.unregister()
    assert node.destroyed == node.subscriptions


def collector(node):
    logger = SimpleNamespace(info=lambda *_: None, warning=lambda *_: None, debug=lambda *_: None)
    return TfTreeCollector(node=node, event_callback=lambda _e: None, logger=logger)


def test_collector_moves_to_a_namespace_once_its_tf_is_published():
    node = FakeNode([], topics={'/tf': TF})
    tf_tree = collector(node)
    assert tf_tree._ensure_listener()
    assert tf_tree._tf_listener.namespace == ''

    node.topics['/robot1/tf'] = TF
    node.published.add('/robot1/tf')
    tf_tree.follow_published_tf()
    assert tf_tree._tf_listener.namespace == '/robot1'
    assert [s.topic for s in node.destroyed] == ['/tf', '/tf_static']
    tf_topics = [s.topic for s in node.subscriptions if s.topic != '/clock']
    assert tf_topics[2:] == ['/robot1/tf', '/robot1/tf_static']
    # The simulation's clock is subscribed once, and kept across the move.
    assert [s.topic for s in node.subscriptions].count('/clock') == 1


def test_collector_stays_on_tf_that_is_still_published():
    node = FakeNode(['/robot1/tf'])
    tf_tree = collector(node)
    assert tf_tree._ensure_listener()
    node.topics['/robot2/tf'] = TF
    node.published.add('/robot2/tf')
    tf_tree.follow_published_tf()
    assert tf_tree._tf_listener.namespace == '/robot1'
    assert node.destroyed == []
