"""A topic_data header and its binary frame always reach the gateway together, on one connection."""

import asyncio
from types import SimpleNamespace

import pytest

pytest.importorskip('websockets')

from websockets.exceptions import ConnectionClosed  # noqa: E402
from websockets.frames import Close  # noqa: E402

from osiris_agent.connection import ConnectionMixin  # noqa: E402

HEADER = '{"type": "topic_data", "topic": "/camera"}'
PAYLOAD = b'\x00\x01\x02'


class FakeAgent(ConnectionMixin):
    def __init__(self):
        log = lambda *a, **k: None  # noqa: E731
        self.logger = SimpleNamespace(info=log, warning=log, error=log)

    def get_logger(self):
        return self.logger


class FakeSocket:
    """Records the frames it sends. `fail_on` makes the send of that frame
    raise as if the gateway had closed; `hang_on` makes it never finish."""

    def __init__(self, fail_on=None, hang_on=None):
        self.sent = []
        self.fail_on = fail_on
        self.hang_on = hang_on
        self.closed = False

    async def close(self):
        self.closed = True

    async def send(self, frame):
        if frame == self.fail_on:
            raise ConnectionClosed(Close(1001, 'going away'), None)
        if frame == self.hang_on:
            await asyncio.Event().wait()
        self.sent.append(frame)


async def drain(agent, ws):
    """Run a send loop on `ws` until the queue is empty, then stop it."""
    task = asyncio.create_task(agent._send_loop(ws))
    while not agent._send_queue.empty():
        await asyncio.sleep(0)
    await asyncio.sleep(0)
    task.cancel()
    await asyncio.gather(task, return_exceptions=True)


def test_a_pair_goes_out_back_to_back():
    async def run():
        agent = FakeAgent()
        agent._send_queue = asyncio.Queue()
        await agent._send_queue.put('before')
        await agent._put_pair(HEADER, PAYLOAD)
        await agent._send_queue.put('after')
        ws = FakeSocket()
        await drain(agent, ws)
        return ws.sent

    assert asyncio.run(run()) == ['before', HEADER, PAYLOAD, 'after']


def test_a_close_while_sending_the_header_leaves_nothing_for_the_next_connection():
    async def run():
        agent = FakeAgent()
        agent._send_queue = asyncio.Queue()
        await agent._put_pair(HEADER, PAYLOAD)
        await agent._send_loop(FakeSocket(fail_on=HEADER))
        await agent._send_queue.put('agent_version')
        ws = FakeSocket()
        await drain(agent, ws)
        return ws.sent

    assert asyncio.run(run()) == ['agent_version']


def test_a_close_between_header_and_frame_leaves_nothing_for_the_next_connection():
    async def run():
        agent = FakeAgent()
        agent._send_queue = asyncio.Queue()
        await agent._put_pair(HEADER, PAYLOAD)
        first = FakeSocket(hang_on=PAYLOAD)
        task = asyncio.create_task(agent._send_loop(first))
        while not first.sent:
            await asyncio.sleep(0)
        task.cancel()
        await asyncio.gather(task, return_exceptions=True)
        await agent._send_queue.put('agent_version')
        second = FakeSocket()
        await drain(agent, second)
        return first.sent, second.sent

    assert asyncio.run(run()) == ([HEADER], ['agent_version'])
