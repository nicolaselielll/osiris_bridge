import math

import pytest

from osiris_agent.command_policy import (
    DEFAULT_MAX_ANGLE_DEG,
    DEFAULT_MAX_DISTANCE_M,
    DEFAULT_MAX_SPEED_MPS,
    DEFAULT_MAX_TIME_ALLOWANCE_S,
    CommandLimits,
    check_command_params,
    refusal_message,
)

LIMITS = CommandLimits()


def test_default_limits():
    assert LIMITS.max_speed_mps == DEFAULT_MAX_SPEED_MPS == 0.5
    assert LIMITS.max_distance_m == DEFAULT_MAX_DISTANCE_M == 5.0
    assert LIMITS.max_angle_deg == DEFAULT_MAX_ANGLE_DEG == 360.0
    assert LIMITS.max_time_allowance_s == DEFAULT_MAX_TIME_ALLOWANCE_S == 120.0


def test_drive_within_limits_is_accepted():
    params = {'distance_m': 1.0, 'speed_mps': 0.15, 'time_allowance_s': 60}
    assert check_command_params('drive', params, LIMITS) is None


def test_drive_with_no_params_uses_safe_defaults():
    assert check_command_params('drive', {}, LIMITS) is None


def test_drive_backwards_is_accepted_within_distance_limit():
    assert check_command_params('drive', {'distance_m': -2.0}, LIMITS) is None


@pytest.mark.parametrize('distance', [5.01, -5.01, 100])
def test_drive_distance_over_limit_is_rejected(distance):
    reason = check_command_params('drive', {'distance_m': distance}, LIMITS)
    assert reason == 'limit_exceeded:distance_m'


@pytest.mark.parametrize('speed', [0.51, 2, 1000])
def test_drive_speed_over_limit_is_rejected(speed):
    reason = check_command_params('drive', {'speed_mps': speed}, LIMITS)
    assert reason == 'limit_exceeded:speed_mps'


@pytest.mark.parametrize('speed', [0, -0.1])
def test_drive_speed_must_be_positive(speed):
    reason = check_command_params('drive', {'speed_mps': speed}, LIMITS)
    assert reason == 'invalid_params:speed_mps'


def test_omitted_speed_is_checked_at_the_default_the_goal_will_use():
    slow_robot = CommandLimits(max_speed_mps=0.1)
    reason = check_command_params('drive', {'distance_m': 1.0}, slow_robot)
    assert reason == 'limit_exceeded:speed_mps'


def test_spin_within_limits_is_accepted():
    assert check_command_params('spin', {'target_yaw_rad': math.pi / 2}, LIMITS) is None


def test_spin_full_turn_is_accepted_in_either_direction():
    assert check_command_params('spin', {'target_yaw_rad': 2 * math.pi}, LIMITS) is None
    assert check_command_params('spin', {'target_yaw_rad': -2 * math.pi}, LIMITS) is None


@pytest.mark.parametrize('yaw', [2 * math.pi + 0.1, -7.0, 50])
def test_spin_over_angle_limit_is_rejected(yaw):
    reason = check_command_params('spin', {'target_yaw_rad': yaw}, LIMITS)
    assert reason == 'limit_exceeded:target_yaw_rad'


@pytest.mark.parametrize('command', ['spin', 'drive'])
@pytest.mark.parametrize('allowance', [120.01, 3600])
def test_time_allowance_over_limit_is_rejected(command, allowance):
    reason = check_command_params(command, {'time_allowance_s': allowance}, LIMITS)
    assert reason == 'limit_exceeded:time_allowance_s'


@pytest.mark.parametrize('command', ['spin', 'drive'])
@pytest.mark.parametrize('allowance', [0, -5])
def test_time_allowance_must_be_positive(command, allowance):
    # Nav2 treats a zero time allowance as unbounded.
    reason = check_command_params(command, {'time_allowance_s': allowance}, LIMITS)
    assert reason == 'invalid_params:time_allowance_s'


@pytest.mark.parametrize('bad', [float('nan'), float('inf'), -float('inf'), 'fast', None, True, [1], {}])
def test_non_numeric_or_non_finite_values_are_rejected(bad):
    assert check_command_params('drive', {'distance_m': bad}, LIMITS) == 'invalid_params:distance_m'
    assert check_command_params('drive', {'speed_mps': bad}, LIMITS) == 'invalid_params:speed_mps'
    assert check_command_params('spin', {'target_yaw_rad': bad}, LIMITS) == 'invalid_params:target_yaw_rad'


def test_numeric_strings_are_accepted_like_the_goal_builder_does():
    assert check_command_params('drive', {'distance_m': '1.5'}, LIMITS) is None


def test_custom_limits_are_honoured():
    wide = CommandLimits(max_speed_mps=1.0, max_distance_m=20.0)
    assert check_command_params('drive', {'distance_m': 15, 'speed_mps': 0.9}, wide) is None
    assert check_command_params('drive', {'distance_m': 15, 'speed_mps': 0.9}, LIMITS) is not None


@pytest.mark.parametrize('params', [None, [], 'drive', 5])
def test_params_must_be_an_object(params):
    assert check_command_params('drive', params, LIMITS) == 'invalid_params:params'


def test_unknown_command_is_rejected():
    assert check_command_params('fly', {}, LIMITS) == 'invalid_params:command'


# What a refusal says, in the same words as the gateway's own check of the
# limits a workspace's owners saved.

@pytest.mark.parametrize('command, params, limits, expected', [
    ('spin', {'target_yaw_rad': math.radians(400)}, LIMITS,
     "angle_deg 400 is over this robot's limit of 360 (command_max_angle_deg)"),
    ('spin', {'target_yaw_rad': -math.radians(361.5)}, LIMITS,
     "angle_deg -361.5 is over this robot's limit of 360 (command_max_angle_deg)"),
    ('drive', {'distance_m': -6}, LIMITS,
     "distance_m -6 is over this robot's limit of 5 (command_max_distance_m)"),
    ('drive', {'distance_m': 1, 'speed_mps': 0.8}, LIMITS,
     "speed_mps 0.8 is over this robot's limit of 0.5 (command_max_speed_mps)"),
    ('spin', {'target_yaw_rad': 1, 'time_allowance_s': 200}, LIMITS,
     "time_allowance_s 200 is over this robot's limit of 120 (command_max_time_allowance_s)"),
    ('drive', {'distance_m': 1}, CommandLimits(max_time_allowance_s=30.0),
     "time_allowance_s 60 (the default when none is given) is over this robot's limit of 30 (command_max_time_allowance_s)"),
    ('drive', {'distance_m': 1, 'time_allowance_s': 10}, CommandLimits(max_speed_mps=0.1),
     "speed_mps 0.15 (the default when none is given) is over this robot's limit of 0.1 (command_max_speed_mps)"),
])
def test_a_limit_refusal_names_the_field_as_sent_the_value_and_the_limit(command, params, limits, expected):
    reason = check_command_params(command, params, limits)
    assert refusal_message(command, params, limits, reason) == expected


@pytest.mark.parametrize('command, params, expected', [
    ('drive', {'distance_m': 'far'}, 'distance_m must be a number'),
    ('drive', {'distance_m': 1, 'speed_mps': -1}, 'speed_mps must be a number above 0'),
    ('spin', {'target_yaw_rad': None}, 'angle_deg must be a number'),
    ('spin', {'time_allowance_s': 0}, 'time_allowance_s must be a number above 0'),
    ('drive', [], 'params must be an object'),
    ('teleport', {}, 'command must be spin or drive'),
])
def test_an_invalid_value_refusal_says_what_is_expected(command, params, expected):
    reason = check_command_params(command, params, LIMITS)
    assert refusal_message(command, params, LIMITS, reason) == expected


def test_other_reasons_have_no_message():
    assert refusal_message('drive', {}, LIMITS, 'commands_disabled') is None
    assert refusal_message('drive', {}, LIMITS, 'goals_disabled') is None
