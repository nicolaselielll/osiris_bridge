"""Subscription QoS that every publisher of a topic can actually deliver to."""

from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

# For status topics (battery, dock) subscribed before their publisher may exist:
# a BEST_EFFORT subscriber receives from both RELIABLE and BEST_EFFORT
# publishers, and an occasional lost status message doesn't matter.
STATUS_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
)


def matching_qos(node, topic: str, depth: int = 10) -> QoSProfile:
    """QoS for subscribing to `topic` that its current publishers can deliver to.

    A RELIABLE subscriber gets nothing from a BEST_EFFORT publisher (most LiDAR
    drivers, many cameras, every simulator), so BEST_EFFORT is used as soon as
    one publisher is. TRANSIENT_LOCAL is used only when every publisher is
    latched: it hands over the message a latched topic such as /map already
    published, but a TRANSIENT_LOCAL subscriber gets nothing from a VOLATILE
    publisher. With no publisher yet, RELIABLE and VOLATILE apply.
    """
    publishers = node.get_publishers_info_by_topic(topic)
    best_effort = any(p.qos_profile.reliability == ReliabilityPolicy.BEST_EFFORT for p in publishers)
    latched = bool(publishers) and all(
        p.qos_profile.durability == DurabilityPolicy.TRANSIENT_LOCAL for p in publishers
    )
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=depth,
        reliability=ReliabilityPolicy.BEST_EFFORT if best_effort else ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL if latched else DurabilityPolicy.VOLATILE,
    )


def qos_key(profile: QoSProfile) -> tuple:
    """The parts of a QoS that decide whether a publisher can deliver to it."""
    return (profile.reliability, profile.durability)
