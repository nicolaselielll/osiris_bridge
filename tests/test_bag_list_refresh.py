"""Stopping a recording sends the robot's bag list again, with the new bag in
it: the gateway keeps that list for the pages that open later."""

import asyncio
import json
import threading
from types import SimpleNamespace

import pytest

pytest.importorskip('websockets')

from osiris_agent.bag_recorder import BagRecorderMixin  # noqa: E402
from osiris_agent.connection import ConnectionMixin  # noqa: E402


class FakeRecording:
    """A `ros2 bag record` process that exits when interrupted."""

    pid = 4242

    def __init__(self):
        self.running = True

    def poll(self):
        return None if self.running else 0

    def send_signal(self, sig):
        self.running = False

    def wait(self, timeout=None):
        return 0


class FakeAgent(ConnectionMixin, BagRecorderMixin):
    def __init__(self, bag_dir, recording_path):
        log = lambda *a, **k: None  # noqa: E731
        self.logger = SimpleNamespace(info=log, warning=log, error=log, debug=log)
        self.context = SimpleNamespace(ok=lambda: True)
        self._bag_dir = str(bag_dir)
        self._bag_lock = threading.Lock()
        self._bag_proc = FakeRecording()
        self._bag_output_path = str(recording_path)
        self.ws = object()
        self.loop = None
        self._send_queue = None

    def get_logger(self):
        return self.logger

    def get_parameter(self, name):
        assert name == 'bag_output_dir'
        return SimpleNamespace(get_parameter_value=lambda: SimpleNamespace(string_value=self._bag_dir))


class FakeSocket:
    """Delivers the given gateway messages, then ends."""

    def __init__(self, *messages):
        self.messages = [json.dumps(m) for m in messages]

    async def _deliver(self):
        for message in self.messages:
            yield message

    def __aiter__(self):
        return self._deliver()


def test_stopping_a_recording_sends_the_bag_list_with_the_new_bag(tmp_path):
    older = tmp_path / 'bag_1792000000'
    older.mkdir()
    (older / 'older_0.mcap').write_bytes(b'x' * 10)
    recording = tmp_path / 'bag_1792000100'
    recording.mkdir()
    (recording / 'recording_0.mcap').write_bytes(b'x' * 100)
    # ros2 bag record writes metadata.yaml when a recording ends.
    for bag in (older, recording):
        (bag / 'metadata.yaml').write_bytes(b'')

    async def run():
        agent = FakeAgent(tmp_path, recording)
        agent.loop = asyncio.get_running_loop()
        agent._send_queue = asyncio.Queue()
        await agent._receive_loop(FakeSocket({'type': 'bag_stop_record'}))
        await asyncio.sleep(0.1)
        sent = []
        while not agent._send_queue.empty():
            sent.append(json.loads(agent._send_queue.get_nowait()))
        return sent

    sent = asyncio.run(run())
    assert [m['name'] for m in sent if m['type'] == 'bag_record_stopped'] == ['bag_1792000100']
    [bag_files] = [m for m in sent if m['type'] == 'bag_files']
    assert [(b['name'], b['size_bytes']) for b in bag_files['bags']] == [('bag_1792000000', 10), ('bag_1792000100', 100)]
