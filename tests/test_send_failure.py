"""A failed send ends the connection quietly, so the agent reconnects."""

import asyncio
from types import SimpleNamespace

import pytest

pytest.importorskip('websockets')

from websockets.exceptions import ConnectionClosed  # noqa: E402
from websockets.frames import Close  # noqa: E402

from osiris_agent.connection import ConnectionMixin  # noqa: E402


class FakeAgent(ConnectionMixin):
    def __init__(self):
        self.errors = []
        log = lambda *a, **k: None  # noqa: E731
        self.logger = SimpleNamespace(info=log, warning=log, error=self.errors.append)

    def get_logger(self):
        return self.logger


class FailingSocket:
    def __init__(self, error):
        self.error = error
        self.closed = False

    async def send(self, frame):
        raise self.error

    async def close(self):
        self.closed = True


def send_once(error):
    """Run the send loop on a socket whose send raises `error`; return the
    loop's result, the socket, the logged errors and asyncio's reports."""
    reports = []

    async def run():
        asyncio.get_running_loop().set_exception_handler(lambda loop, context: reports.append(context))
        agent = FakeAgent()
        agent._send_queue = asyncio.Queue()
        await agent._send_queue.put('{"type": "telemetry"}')
        ws = FailingSocket(error)
        task = asyncio.create_task(agent._send_loop(ws))
        result = await asyncio.wait_for(task, 1.0)
        return result, ws, agent.errors

    result, ws, errors = asyncio.run(run())
    return result, ws, errors, reports


@pytest.mark.parametrize('error', [
    ConnectionClosed(Close(1001, 'going away'), None),
    RuntimeError('unexpected'),
])
def test_a_failed_send_closes_the_connection_and_ends_without_an_error(error):
    result, ws, errors, reports = send_once(error)
    assert result is None
    assert ws.closed
    assert len(errors) == 1 and errors[0].startswith('WS send failed')
    assert reports == []
