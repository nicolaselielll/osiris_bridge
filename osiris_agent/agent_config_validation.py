"""Checks a cloud agent_config push before any of it is applied.

A push is applied all or nothing: validate_agent_config either returns every
known field converted to the type the agent uses, or the list of problems.
The rules cover what the agent can apply safely (types, finite numbers,
usable ranges). Product limits such as the top speed the web app allows stay
in the web app, so widening one there never makes the agent refuse a push.
Pure functions with no ROS dependency, so they can be exercised without a ROS
installation.
"""

import math

BT_MODES = ('off', 'nav2', 'btcpp')


def _boolean(value):
    if not isinstance(value, bool):
        raise ValueError('expected true or false')
    return value


def _number(value, *, minimum=None, above=None, maximum=None):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError('expected a number')
    number = float(value)
    if not math.isfinite(number):
        raise ValueError('expected a finite number')
    if above is not None and number <= above:
        raise ValueError(f'must be more than {above:g}')
    if minimum is not None and number < minimum:
        raise ValueError(f'must be at least {minimum:g}')
    if maximum is not None and number > maximum:
        raise ValueError(f'must be at most {maximum:g}')
    return number


def _whole_number(value, **limits):
    number = _number(value, **limits)
    if not number.is_integer():
        raise ValueError('expected a whole number')
    return int(number)


def _text(value, *, allow_empty=False):
    if not isinstance(value, str):
        raise ValueError('expected a string')
    if not value and not allow_empty:
        raise ValueError('must not be empty')
    return value


def _bt_mode(value):
    if value not in BT_MODES:
        raise ValueError(f'expected one of {", ".join(BT_MODES)}')
    return value


def _positive(value):
    return _number(value, above=0)


def _not_negative(value):
    return _number(value, minimum=0)


def _not_negative_whole(value):
    return _whole_number(value, minimum=0)


def _jpeg_quality(value):
    return _whole_number(value, minimum=0, maximum=100)


def _port(value):
    return _whole_number(value, minimum=1, maximum=65535)


FIELDS = {
    'telemetry_enabled': _boolean,
    'goals_enabled': _boolean,
    'params_enabled': _boolean,
    'tf_tree_enabled': _boolean,
    'tf_tree_poll_interval': _positive,
    'ros2_control_enabled': _boolean,
    'ros2_control_poll_interval': _positive,
    'battery_topic': _text,
    'battery_check_enabled': _boolean,
    'battery_min_percent': lambda v: _number(v, minimum=0, maximum=100),
    'dock_check_enabled': _boolean,
    # An empty dock topic means the robot has none.
    'dock_status_topic': lambda v: _text(v, allow_empty=True),
    'commands_enabled': _boolean,
    'command_max_speed_mps': _positive,
    'command_max_distance_m': _positive,
    'command_max_angle_deg': _positive,
    'command_max_time_allowance_s': _positive,
    'bt_mode': _bt_mode,
    'bt_host': _text,
    'bt_server_port': _port,
    'bt_publisher_port': _port,
    'graph_debounce_interval': _not_negative,
    'topic_data_rate_hz': _not_negative,
    'topic_data_max_bytes_per_sec': _not_negative,
    'global_topic_data_max_bytes_per_sec': _not_negative,
    'global_topic_data_max_msgs_per_sec': _not_negative,
    'image_jpeg_quality': _jpeg_quality,
    'image_max_dimension': _not_negative_whole,
    'pointcloud_max_points': _not_negative_whole,
    'pointcloud_keep_intensity': _boolean,
    'pointcloud_keep_rgb': _boolean,
    'pointcloud_default_rate_hz': _not_negative,
}

# Per-topic overrides inside topic_limits, read on every message of their topic.
TOPIC_LIMIT_FIELDS = {
    'rate_hz': _not_negative,
    'max_bytes_per_sec': _not_negative,
    'jpeg_quality': _jpeg_quality,
    'max_dimension': _not_negative_whole,
    'pointcloud_max_points': _not_negative_whole,
    'pointcloud_keep_intensity': _boolean,
    'pointcloud_keep_rgb': _boolean,
}


def _topic_limits(value, problems):
    if value is None:
        return {}
    if not isinstance(value, dict):
        problems.append(f'topic_limits={value!r}: expected an object')
        return {}
    limits = {}
    for topic, entry in value.items():
        if not isinstance(topic, str) or not topic:
            problems.append(f'topic_limits key {topic!r}: expected a topic name')
            continue
        if not isinstance(entry, dict):
            problems.append(f'topic_limits[{topic}]={entry!r}: expected an object')
            continue
        limits[topic] = {}
        for name, check in TOPIC_LIMIT_FIELDS.items():
            if name in entry:
                try:
                    limits[topic][name] = check(entry[name])
                except ValueError as e:
                    problems.append(f'topic_limits[{topic}].{name}={entry[name]!r}: {e}')
    return limits


def validate_agent_config(config):
    """Return (config, problems).

    config holds every known field of the push, converted. Unknown fields are
    left out, so a newer web app's settings never break an older agent.
    problems lists each malformed value as "field=value: reason"; when it is
    not empty, nothing of the push may be applied.
    """
    if not isinstance(config, dict):
        return {}, [f'config={config!r}: expected an object']
    clean, problems = {}, []
    for name, check in FIELDS.items():
        if name in config:
            try:
                clean[name] = check(config[name])
            except ValueError as e:
                problems.append(f'{name}={config[name]!r}: {e}')
    if 'topic_limits' in config:
        clean['topic_limits'] = _topic_limits(config['topic_limits'], problems)
    return clean, problems
