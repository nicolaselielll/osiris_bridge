import base64
import importlib.util
import sys
import types

import pytest


def _installed(name):
    # A stand-in another test module installed has no spec.
    module = sys.modules.get(name)
    if module is not None:
        return module.__spec__ is not None
    return importlib.util.find_spec(name) is not None


def _install_ros_stand_ins():
    # snapshots.py imports rclpy and rosidl_runtime_py at module level. Where
    # ROS is not installed, minimal stand-ins let it be imported; the message
    # conversion it uses is replaced in each test anyway.
    qos = types.ModuleType('rclpy.qos')
    qos.QoSProfile = lambda **fields: types.SimpleNamespace(**fields)
    qos.DurabilityPolicy = types.SimpleNamespace(VOLATILE='volatile', TRANSIENT_LOCAL='transient_local')
    qos.HistoryPolicy = types.SimpleNamespace(KEEP_LAST='keep_last')
    qos.ReliabilityPolicy = types.SimpleNamespace(RELIABLE='reliable', BEST_EFFORT='best_effort')
    rclpy = sys.modules.get('rclpy') or types.ModuleType('rclpy')
    rclpy.qos = qos
    utilities = types.ModuleType('rosidl_runtime_py.utilities')
    utilities.get_message = lambda name: None
    rosidl = types.ModuleType('rosidl_runtime_py')
    rosidl.message_to_ordereddict = dict
    rosidl.utilities = utilities
    sys.modules.update({
        'rclpy': rclpy, 'rclpy.qos': qos,
        'rosidl_runtime_py': rosidl, 'rosidl_runtime_py.utilities': utilities,
    })


if not _installed('rclpy'):
    _install_ros_stand_ins()

from osiris_agent import snapshots  # noqa: E402
from osiris_agent.payload_encoding import BINARY_MARKER_KEY, PayloadEncodingMixin  # noqa: E402

STRING = ('std_msgs/msg/String', {'data': 'hello'})
FLOAT = ('std_msgs/msg/Float32', {'data': 1.5})


class _Parameter:
    def __init__(self, value):
        self._value = value

    def get_parameter_value(self):
        return types.SimpleNamespace(integer_value=self._value, bool_value=self._value, double_value=self._value)


class FakeAgent(snapshots.SnapshotMixin, PayloadEncodingMixin):
    """The parts of the agent node a bundle capture uses, with every capture
    delivering its topic's message immediately."""

    PARAMETERS = {'image_jpeg_quality': 80, 'image_max_dimension': 0}

    def __init__(self, topics):
        self.topics = topics  # topic name -> (message type, message)
        self.started = []  # topics a capture was started on
        self.sent = []
        self.warnings = []
        self._image_reencode_unsupported_warned = set()

    def get_topic_names_and_types(self):
        return [(name, [msg_type]) for name, (msg_type, _) in self.topics.items()]

    def get_parameter(self, name):
        return _Parameter(self.PARAMETERS[name])

    def get_logger(self):
        return types.SimpleNamespace(warning=self.warnings.append, info=lambda *a: None, debug=lambda *a: None)

    def _start_one_shot_capture(self, topic_name, on_message, on_timeout, on_not_found, timeout_s=3.0):
        self.started.append(topic_name)
        if topic_name in self.topics:
            on_message(self.topics[topic_name][1])
        else:
            on_not_found()

    def _send_bundle_capture_result(self, request_id, captures):
        self.sent.append((request_id, captures))


@pytest.fixture(autouse=True)
def plain_messages(monkeypatch):
    # Test messages are plain dicts, so converting one is just a copy.
    monkeypatch.setattr(snapshots, 'message_to_ordereddict', dict)


def capture(agent, topics, **request):
    agent._handle_bundle_capture_request({'request_id': 'r1', 'topics': topics, **request})
    assert len(agent.sent) == 1
    request_id, captures = agent.sent[0]
    assert request_id == 'r1'
    return captures


def test_string_topic_is_captured_as_data():
    agent = FakeAgent({'/chatter': STRING})
    assert capture(agent, ['/chatter']) == {'/chatter': {'status': 'success', 'data': {'data': 'hello'}}}


def test_float_topic_is_captured_as_data():
    agent = FakeAgent({'/speed': FLOAT})
    assert capture(agent, ['/speed']) == {'/speed': {'status': 'success', 'data': {'data': 1.5}}}


def test_mixed_bundle_returns_every_capture():
    pytest.importorskip('PIL')
    image = {'height': 2, 'width': 2, 'encoding': 'rgb8', 'is_bigendian': 0, 'step': 6, 'data': list(range(12))}
    agent = FakeAgent({'/camera': ('sensor_msgs/msg/Image', image), '/chatter': STRING, '/speed': FLOAT})

    captures = capture(agent, ['/camera', '/chatter', '/speed', '/missing'])

    assert captures['/chatter'] == {'status': 'success', 'data': {'data': 'hello'}}
    assert captures['/speed'] == {'status': 'success', 'data': {'data': 1.5}}
    assert captures['/missing'] == {'status': 'failed', 'reason': 'topic_not_found'}
    camera = captures['/camera']
    assert camera['status'] == 'success'
    assert camera['data'][BINARY_MARKER_KEY]['format'] == 'jpeg'
    assert camera['data'][BINARY_MARKER_KEY]['encoding'] == 'base64'
    assert base64.b64decode(camera['data']['data']).startswith(b'\xff\xd8')


def test_capture_that_cannot_be_encoded_fails_on_its_own(monkeypatch):
    agent = FakeAgent({'/chatter': STRING, '/broken': ('example_msgs/msg/Broken', {'data': 1})})
    encode = agent._encode_bundle_capture_data

    def failing_encode(name, msg):
        if name == '/broken':
            raise ValueError('cannot encode')
        return encode(name, msg)

    monkeypatch.setattr(agent, '_encode_bundle_capture_data', failing_encode)
    captures = capture(agent, ['/chatter', '/broken'])

    assert captures['/broken'] == {'status': 'failed', 'reason': 'encode_failed'}
    assert captures['/chatter'] == {'status': 'success', 'data': {'data': 'hello'}}
    assert any('/broken' in w for w in agent.warnings)


class _UnreadableScan:
    @property
    def header(self):
        raise RuntimeError('corrupt scan')


def test_unreadable_scan_fails_only_the_wall_fit():
    agent = FakeAgent({'/chatter': STRING, '/scan': ('sensor_msgs/msg/LaserScan', _UnreadableScan())})
    captures = capture(agent, ['/chatter'], lidar_wall_fit={})

    assert captures['lidar_wall_fit'] == {'status': 'failed', 'reason': 'encode_failed'}
    assert captures['/chatter'] == {'status': 'success', 'data': {'data': 'hello'}}


class _Scan(dict):
    """A LaserScan as the wall-fit capture reads it: its fields and a header."""
    header = types.SimpleNamespace(frame_id='laser')


def test_wall_fit_on_a_laserscan_returns_the_raw_scan():
    scan = _Scan(angle_min=-0.5, angle_max=0.5, ranges=[2.0, 2.0, 2.0])
    agent = FakeAgent({'/scan': ('sensor_msgs/msg/LaserScan', scan)})
    captures = capture(agent, [], lidar_wall_fit={})

    assert captures['lidar_wall_fit']['status'] == 'success'
    assert captures['lidar_wall_fit']['raw_scan']['ranges'] == [2.0, 2.0, 2.0]


def test_wall_fit_on_a_topic_that_is_not_a_laserscan():
    agent = FakeAgent({'/chatter': STRING, '/camera': ('sensor_msgs/msg/Image', _Scan(data=[0]))})
    captures = capture(agent, ['/chatter'], lidar_wall_fit={'topic': '/camera'})

    # Same reason the dedicated lidar snapshot gives, and no subscription is made.
    assert captures['lidar_wall_fit'] == {'status': 'failed', 'reason': 'not_a_laserscan'}
    assert captures['/chatter'] == {'status': 'success', 'data': {'data': 'hello'}}
    assert '/camera' not in agent.started


def test_wall_fit_on_a_missing_topic_reports_topic_not_found():
    agent = FakeAgent({'/chatter': STRING})
    captures = capture(agent, ['/chatter'], lidar_wall_fit={'topic': '/missing_scan'})

    assert captures['lidar_wall_fit'] == {'status': 'failed', 'reason': 'topic_not_found'}
    assert captures['/chatter'] == {'status': 'success', 'data': {'data': 'hello'}}


@pytest.mark.parametrize('msg_type, data', [
    STRING,
    FLOAT,
    ('std_msgs/msg/Int32MultiArray', {'data': [1, 2, 3]}),
    ('nav_msgs/msg/OccupancyGrid', {'data': [-1, 0, 100]}),
])
def test_only_images_are_base64_encoded(msg_type, data):
    assert FakeAgent({})._capture_image_as_base64('/topic', dict(data), msg_type) is None
