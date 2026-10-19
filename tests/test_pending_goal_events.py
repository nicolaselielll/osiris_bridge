"""Goal events from while the agent isn't connected are kept and sent on the
next connection, after the initial state; other events are dropped as before."""

import asyncio
import json
from types import SimpleNamespace

import pytest

pytest.importorskip('websockets')

from osiris_agent import connection  # noqa: E402
from osiris_agent.connection import ConnectionMixin  # noqa: E402


class FakeAgent(ConnectionMixin):
    def __init__(self):
        self.ws = None
        self.loop = None
        log = lambda *a, **k: None  # noqa: E731
        self.logger = SimpleNamespace(info=log, warning=log, error=log)

    def get_logger(self):
        return self.logger


def goal(goal_id, status, **extra):
    return {'type': 'goal_event', 'action': '/spin', 'goal_id': goal_id, 'status': status, 'timestamp': 1.0, **extra}


def drain(agent):
    async def run():
        agent._send_queue = asyncio.Queue()
        await agent._send_pending_goal_events()
        out = []
        while not agent._send_queue.empty():
            out.append(json.loads(agent._send_queue.get_nowait()))
        return out
    return asyncio.run(run())


def test_goal_events_while_disconnected_are_sent_on_the_next_connection_in_order():
    agent = FakeAgent()
    agent._enqueue(goal('a', 2))
    agent._enqueue(goal('a', 4))
    agent._enqueue(goal('b', 4, replayed=True))
    sent = drain(agent)
    assert [(e['goal_id'], e['status'], e.get('replayed', False)) for e in sent] == [('a', 2, False), ('a', 4, False), ('b', 4, True)]
    assert drain(agent) == [], 'sent once, not again on the connection after'


def test_other_events_while_disconnected_are_dropped():
    agent = FakeAgent()
    agent._enqueue({'type': 'topic_data', 'topic': '/odom', 'data': {}})
    agent._enqueue({'type': 'action_feedback', 'action': '/spin', 'goal_id': 'a', 'feedback': {}})
    assert drain(agent) == []


def test_a_goal_event_while_the_kept_ones_wait_goes_behind_them():
    # Connected but the initial state isn't sent yet: a new status is kept
    # behind the earlier ones, so a goal never goes back to running.
    agent = FakeAgent()
    agent.ws, agent.loop = object(), object()
    agent._enqueue(goal('a', 2))
    agent._enqueue(goal('a', 4))
    assert [e['status'] for e in drain(agent)] == [2, 4]


def test_after_the_drain_goal_events_go_straight_out_until_the_connection_ends():
    agent = FakeAgent()
    agent._enqueue(goal('a', 2))
    async def run():
        agent._send_queue = asyncio.Queue()
        agent.loop = asyncio.get_running_loop()
        agent.ws = object()
        await agent._send_pending_goal_events()
        agent._enqueue(goal('a', 4))
        for _ in range(5):
            await asyncio.sleep(0)
        statuses = []
        while not agent._send_queue.empty():
            statuses.append(json.loads(agent._send_queue.get_nowait())['status'])
        return statuses
    assert asyncio.run(run()) == [2, 4]
    agent._stop_goal_events_live()
    agent.ws = None
    agent._enqueue(goal('b', 4))
    assert [e['goal_id'] for e in drain(agent)] == ['b'], 'kept again once the connection ended'


def test_only_the_newest_are_kept_past_the_limit(monkeypatch):
    monkeypatch.setattr(connection, 'PENDING_GOAL_EVENTS_MAX', 3)
    agent = FakeAgent()
    for i in range(5):
        agent._enqueue(goal(str(i), 4))
    assert [e['goal_id'] for e in drain(agent)] == ['2', '3', '4']
