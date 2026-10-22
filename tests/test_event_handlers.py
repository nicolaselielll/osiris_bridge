"""The bridge's subscriptions and publishers carry no default QoS event handlers:
the executor would check each of them again on every message."""
import pytest

pytest.importorskip('rclpy')

from rclpy.node import Node  # noqa: E402

from osiris_agent.agent_node import WebBridge  # noqa: E402


def test_subscriptions_and_publishers_skip_rclpys_default_event_handlers(monkeypatch):
    calls = []
    monkeypatch.setattr(Node, 'create_subscription', lambda self, *args, **kwargs: calls.append(kwargs))
    monkeypatch.setattr(Node, 'create_publisher', lambda self, *args, **kwargs: calls.append(kwargs))
    bridge = object.__new__(WebBridge)
    bridge.create_subscription('Msg', '/topic', lambda _msg: None, 10)
    bridge.create_publisher('Msg', '/topic', 10)
    assert [kwargs['event_callbacks'].use_default_callbacks for kwargs in calls] == [False, False]

    # Callbacks a caller passes are kept.
    mine = object()
    bridge.create_subscription('Msg', '/topic', lambda _msg: None, 10, event_callbacks=mine)
    assert calls[-1]['event_callbacks'] is mine
