"""The agent backs off between reconnect attempts and starts over after a healthy connection."""

import asyncio
from types import SimpleNamespace

import pytest

pytest.importorskip('websockets')

from osiris_agent import connection  # noqa: E402
from osiris_agent.connection import RECONNECT_INITIAL_DELAY, RECONNECT_MAX_DELAY, ConnectionMixin  # noqa: E402


class FakeAgent(ConnectionMixin):
    """Runs the reconnect loop over a list of attempts: None fails before
    authenticating, a number is a connection that stays up that many seconds."""

    def __init__(self, attempts, clock):
        self.attempts = list(attempts)
        self.clock = clock
        self.context = SimpleNamespace(ok=lambda: bool(self.attempts))
        log = lambda *a, **k: None  # noqa: E731
        self.logger = SimpleNamespace(info=log, warning=log, error=log)

    def get_logger(self):
        return self.logger

    async def _client_loop(self):
        session = self.attempts.pop(0)
        if session is None:
            raise OSError('connection refused')
        self._connected_since = self.clock.now
        self.clock.now += session


def waits_for(attempts, monkeypatch, jitter=1.0):
    clock = SimpleNamespace(now=1000.0)
    waits = []

    async def fake_sleep(seconds):
        waits.append(seconds)
        clock.now += seconds

    monkeypatch.setattr(connection.time, 'monotonic', lambda: clock.now)
    monkeypatch.setattr(connection.asyncio, 'sleep', fake_sleep)
    monkeypatch.setattr(connection.random, 'uniform', lambda low, high: jitter)
    asyncio.run(FakeAgent(attempts, clock)._client_loop_with_reconnect())
    return waits


def test_failed_attempts_back_off_to_the_longest_wait(monkeypatch):
    assert waits_for([None] * 8, monkeypatch) == [1, 2, 4, 8, 16, 30, 30, 30]


def test_a_long_connection_starts_the_waits_over(monkeypatch):
    attempts = [None] * 6 + [RECONNECT_MAX_DELAY, None, None]
    assert waits_for(attempts, monkeypatch) == [1, 2, 4, 8, 16, 30, RECONNECT_INITIAL_DELAY, 2, 4]


def test_short_connections_keep_backing_off(monkeypatch):
    assert waits_for([5.0] * 7, monkeypatch) == [1, 2, 4, 8, 16, 30, 30]


def test_every_wait_has_jitter(monkeypatch):
    assert waits_for([None, None], monkeypatch, jitter=1.25) == [1.25, 2.5]
