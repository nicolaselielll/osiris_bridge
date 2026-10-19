"""The agent logs why the gateway turned it away, and what to do about it."""

from types import SimpleNamespace

import pytest

pytest.importorskip('websockets')

from websockets.exceptions import ConnectionClosed  # noqa: E402
from websockets.frames import Close  # noqa: E402

from osiris_agent.connection import ConnectionMixin  # noqa: E402


class FakeAgent(ConnectionMixin):
    def __init__(self):
        self.errors = []

    def get_logger(self):
        return SimpleNamespace(error=self.errors.append)


def refusal(code, reason):
    agent = FakeAgent()
    agent._log_gateway_refusal(ConnectionClosed(Close(code, reason), None))
    return agent.errors[0]


def test_a_full_workspace_says_so_and_how_to_free_space():
    message = refusal(4429, 'Storage limit reached')
    assert message.startswith('Gateway refused the connection: Storage limit reached (code 4429): ')
    assert 'Delete data' in message


def test_a_bad_token_points_to_rotating_it():
    assert 'Rotate it in the web app' in refusal(1008, 'Invalid token')


def test_an_unknown_code_still_gives_the_reason():
    assert refusal(4000, 'Robot state deleted') == 'Gateway refused the connection: Robot state deleted (code 4000)'
