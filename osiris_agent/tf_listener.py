"""TF for the agent's own lookups, read from wherever the robot publishes it.

tf2_ros's TransformListener always reads /tf and /tf_static. A Nav2 brought up
under a namespace (nav2_bringup's use_namespace) remaps them to <ns>/tf and
<ns>/tf_static, so an agent at the root would see no frames at all.

ROS packages are imported where they are used, so the capture code that
imports this module still loads where ROS isn't installed.
"""

TF_TYPE = 'tf2_msgs/msg/TFMessage'


def tf_qos(static: bool = False):
    """The QoS tf2_ros's own listener uses for tf, or with static for tf_static."""
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile
    return QoSProfile(
        depth=100,
        durability=DurabilityPolicy.TRANSIENT_LOCAL if static else DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
    )


def published_tf_namespaces(node) -> list:
    """Namespaces ('' for the root) with a published tf or tf_static topic.
    Only published topics count: the agent's own subscription makes a TF topic
    exist without a publisher."""
    found = set()
    for name, types in node.get_topic_names_and_types():
        if TF_TYPE not in types:
            continue
        for suffix in ('/tf', '/tf_static'):
            if name.endswith(suffix) and node.count_publishers(name) > 0:
                found.add(name[:-len(suffix)])
    return sorted(found)


def resolve_tf_namespace(node) -> str:
    """Namespace whose TF to read: the root when TF is published there, else
    this node's own namespace if it has TF, else the only namespace with TF,
    else the root. With several robots' TF and none in this node's namespace,
    which robot is this one is not something to guess."""
    candidates = published_tf_namespaces(node)
    if '' in candidates:
        return ''
    own = node.get_namespace().rstrip('/')
    if own in candidates:
        return own
    if len(candidates) == 1:
        return candidates[0]
    return ''


def tf_namespace_published(node, namespace: str) -> bool:
    return any(node.count_publishers(f'{namespace}/{name}') > 0 for name in ('tf', 'tf_static'))


class TfListener:
    """Feeds a tf2 Buffer from <namespace>/tf and <namespace>/tf_static, as
    tf2_ros.TransformListener does from /tf and /tf_static."""

    def __init__(self, buffer, node, namespace: str = ''):
        from rclpy.callback_groups import ReentrantCallbackGroup
        from tf2_msgs.msg import TFMessage

        self.buffer = buffer
        self.namespace = namespace
        self._node = node
        # Reentrant like tf2_ros's listener, so a lookup in another callback
        # never waits on these.
        group = ReentrantCallbackGroup()
        self._subs = [
            node.create_subscription(TFMessage, f'{namespace}/tf', self._on_tf, tf_qos(), callback_group=group),
            node.create_subscription(TFMessage, f'{namespace}/tf_static', self._on_tf_static, tf_qos(static=True), callback_group=group),
        ]

    def _on_tf(self, msg):
        for transform in msg.transforms:
            self.buffer.set_transform(transform, 'default_authority')

    def _on_tf_static(self, msg):
        for transform in msg.transforms:
            self.buffer.set_transform_static(transform, 'default_authority')

    def unregister(self):
        for sub in self._subs:
            self._node.destroy_subscription(sub)
        self._subs = []
