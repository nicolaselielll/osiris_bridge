"""A cloud agent_config push is checked as a whole before any of it is applied."""

import copy
import math

import pytest

from osiris_agent.agent_config_validation import validate_agent_config

# Realistic agent_config pushes with the full set of fields.
CONFIG_A = {
    'telemetry_enabled': True, 'goals_enabled': True, 'params_enabled': True,
    'tf_tree_enabled': True, 'tf_tree_poll_interval': 0.5,
    'ros2_control_enabled': False, 'ros2_control_poll_interval': 2.0,
    'battery_topic': '/battery_state', 'battery_check_enabled': True, 'battery_min_percent': 30.0,
    'dock_check_enabled': False, 'dock_status_topic': '/dock_status',
    'commands_enabled': True, 'command_max_speed_mps': 0.5, 'command_max_distance_m': 5.0,
    'command_max_angle_deg': 360.0, 'command_max_time_allowance_s': 60.0,
    'bt_mode': 'nav2', 'bt_host': '127.0.0.1', 'bt_server_port': 1667, 'bt_publisher_port': 1668,
    'graph_debounce_interval': 0.5,
    'topic_data_rate_hz': 20.0, 'topic_data_max_bytes_per_sec': 0.0,
    'global_topic_data_max_bytes_per_sec': 0.0, 'global_topic_data_max_msgs_per_sec': 0.0,
    'image_jpeg_quality': 60, 'image_max_dimension': 32,
    'pointcloud_max_points': 500, 'pointcloud_keep_intensity': True, 'pointcloud_keep_rgb': False,
    'pointcloud_default_rate_hz': 2.0,
    'topic_limits': {
        '/fake/string': {'rate_hz': 2.0},
        '/fake/cloud': {'pointcloud_keep_rgb': True, 'pointcloud_max_points': 300},
        '/fake/image_bgra': {'jpeg_quality': 80, 'max_dimension': 0},
        '/fake/grid': {'max_bytes_per_sec': 50.0},
    },
}
CONFIG_B = {**copy.deepcopy(CONFIG_A), 'battery_min_percent': 90.0, 'global_topic_data_max_msgs_per_sec': 3.0}
CONFIG_C = {**copy.deepcopy(CONFIG_A), 'battery_check_enabled': False, 'dock_check_enabled': True,
            'tf_tree_poll_interval': 1.0, 'ros2_control_enabled': True}
CONFIG_D = {**copy.deepcopy(CONFIG_A), 'telemetry_enabled': False, 'tf_tree_enabled': False,
            'goals_enabled': False, 'params_enabled': False, 'bt_mode': 'off', 'commands_enabled': False,
            'ros2_control_enabled': False, 'topic_limits': {}}


def problems_for(config):
    return validate_agent_config(config)[1]


@pytest.mark.parametrize('config', [CONFIG_A, CONFIG_B, CONFIG_C, CONFIG_D, {}])
def test_valid_configs_pass_unchanged(config):
    clean, problems = validate_agent_config(config)
    assert problems == []
    assert clean == config


def test_the_bag_folder_is_not_a_cloud_setting():
    # It's set on the robot only: a pushed value, even a malformed one, is
    # left out like an unknown field.
    for value in ('/home/robot', ['/tmp']):
        clean, problems = validate_agent_config({'telemetry_enabled': False, 'bag_output_dir': value})
        assert problems == []
        assert clean == {'telemetry_enabled': False}


def test_unknown_fields_are_left_out():
    clean, problems = validate_agent_config({'telemetry_enabled': False, 'from_a_newer_web_app': 3})
    assert problems == []
    assert clean == {'telemetry_enabled': False}


def test_whole_numbers_become_ints_and_numbers_floats():
    clean, _ = validate_agent_config({'bt_server_port': 1667.0, 'tf_tree_poll_interval': 1})
    assert clean == {'bt_server_port': 1667, 'tf_tree_poll_interval': 1.0}
    assert type(clean['bt_server_port']) is int and type(clean['tf_tree_poll_interval']) is float


@pytest.mark.parametrize('field, value', [
    ('telemetry_enabled', 'false'),
    ('commands_enabled', 1),
    ('pointcloud_keep_rgb', None),
    ('battery_min_percent', 'not-a-number'),
    ('battery_min_percent', True),
    ('battery_min_percent', 150),
    ('topic_data_rate_hz', math.inf),
    ('topic_data_rate_hz', math.nan),
    ('topic_data_rate_hz', -1),
    ('tf_tree_poll_interval', 0),
    ('ros2_control_poll_interval', -2.0),
    ('command_max_speed_mps', 0),
    ('image_jpeg_quality', 101),
    ('image_jpeg_quality', 7.5),
    ('pointcloud_max_points', -10),
    ('bt_server_port', 0),
    ('bt_publisher_port', 70000),
    ('bt_mode', 'nav'),
    ('bt_host', ''),
    ('battery_topic', 5),
    ('topic_limits', 'none'),
    ('topic_limits', {'/fake/string': 2.0}),
    ('topic_limits', {'': {'rate_hz': 2.0}}),
    ('topic_limits', {'/fake/string': {'rate_hz': 'fast'}}),
    ('topic_limits', {'/fake/image': {'jpeg_quality': 0.5}}),
    ('topic_limits', {'/fake/cloud': {'pointcloud_keep_rgb': 'yes'}}),
])
def test_each_kind_of_malformed_value_is_named(field, value):
    problems = problems_for({**CONFIG_A, field: value})
    assert len(problems) == 1
    assert problems[0].startswith(field)


def test_every_bad_field_is_listed():
    problems = problems_for({**CONFIG_A, 'telemetry_enabled': 'yes', 'bt_server_port': 'x'})
    assert [p.split('=')[0] for p in problems] == ['telemetry_enabled', 'bt_server_port']


def test_a_push_that_is_not_an_object_is_malformed():
    assert problems_for(['telemetry_enabled']) == ["config=['telemetry_enabled']: expected an object"]


def test_an_empty_dock_topic_means_no_dock():
    assert problems_for({'dock_status_topic': ''}) == []


def test_null_topic_limits_means_none():
    assert validate_agent_config({'topic_limits': None}) == ({'topic_limits': {}}, [])
