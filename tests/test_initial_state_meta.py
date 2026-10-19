"""The agent says which ROS node it is, so its robot's Nav2 can be found among several."""

import pytest

pytest.importorskip('psutil')

from osiris_agent.telemetry import TelemetryMixin  # noqa: E402


class FakeAgent(TelemetryMixin):
    def __init__(self, fqn):
        self.fqn = fqn

    def get_fully_qualified_name(self):
        return self.fqn

    def _get_cpu_model(self):
        return None

    def _get_robot_model(self):
        return None


@pytest.mark.parametrize('fqn', ['/osiris_node', '/robot1/osiris_node'])
def test_meta_names_the_agents_node(fqn):
    meta = FakeAgent(fqn)._get_initial_state_meta({'ram': {'total_mb': 1024.0}})
    assert meta['agent_node'] == fqn
