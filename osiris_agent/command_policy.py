"""Limits applied to robot motion commands before they reach Nav2.

Motion commands (spin, drive) are only accepted when the agent has commands
enabled, and every accepted command must stay within the configured limits.
The checks here are pure functions with no ROS dependency, so they can be
exercised without a ROS installation.
"""

import math
from dataclasses import dataclass

# Values used when a command does not specify a field itself.
DEFAULT_DRIVE_SPEED_MPS = 0.15
# Nav2 treats an unset or zero time allowance as unbounded, so every command
# gets a finite one.
DEFAULT_TIME_ALLOWANCE_S = 60.0

# Default limits, applied unless the operator configures different ones.
DEFAULT_MAX_SPEED_MPS = 0.5
DEFAULT_MAX_DISTANCE_M = 5.0
DEFAULT_MAX_ANGLE_DEG = 360.0
DEFAULT_MAX_TIME_ALLOWANCE_S = 120.0


@dataclass(frozen=True)
class CommandLimits:
    max_speed_mps: float = DEFAULT_MAX_SPEED_MPS
    max_distance_m: float = DEFAULT_MAX_DISTANCE_M
    max_angle_deg: float = DEFAULT_MAX_ANGLE_DEG
    max_time_allowance_s: float = DEFAULT_MAX_TIME_ALLOWANCE_S


def _as_finite_number(value):
    """Return value as a float, or None if it is not a finite number."""
    if isinstance(value, bool):
        return None
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    return number if math.isfinite(number) else None


def _check_value(params, field, default, limit, *, allow_negative, unit_scale=1.0):
    """Check one numeric field against a limit.

    Returns a rejection reason, or None if the effective value is acceptable.
    `unit_scale` converts the field's own unit into the limit's unit.
    """
    raw = params.get(field, default)
    number = _as_finite_number(raw)
    if number is None:
        return f'invalid_params:{field}'
    if not allow_negative and number <= 0:
        return f'invalid_params:{field}'
    if abs(number) * unit_scale > limit:
        return f'limit_exceeded:{field}'
    return None


def check_command_params(command, params, limits):
    """Return a rejection reason if the command breaks a limit, else None.

    Reasons take the form `invalid_params:<field>` for values that are not
    usable numbers and `limit_exceeded:<field>` for values beyond the limits.
    Fields a command omits are checked at the default the goal builder will
    use for them, so leaving a field out can never bypass a limit.
    """
    if not isinstance(params, dict):
        return 'invalid_params:params'

    reason = _check_value(
        params, 'time_allowance_s', DEFAULT_TIME_ALLOWANCE_S,
        limits.max_time_allowance_s, allow_negative=False,
    )
    if reason:
        return reason

    if command == 'spin':
        return _check_value(
            params, 'target_yaw_rad', 0.0,
            limits.max_angle_deg, allow_negative=True,
            unit_scale=180.0 / math.pi,
        )

    if command == 'drive':
        # A negative distance drives backwards; speed is always a magnitude.
        return (
            _check_value(
                params, 'distance_m', 0.0,
                limits.max_distance_m, allow_negative=True,
            )
            or _check_value(
                params, 'speed_mps', DEFAULT_DRIVE_SPEED_MPS,
                limits.max_speed_mps, allow_negative=False,
            )
        )

    return 'invalid_params:command'


# How each checked field is given to a person or the AI, its default when a
# command leaves it out, and the setting that limits it. A spin is given in
# degrees (the AI's spin tool and the Actions pane both take angle_deg) and
# reaches the agent in radians.
_FIELDS = {
    'time_allowance_s': ('time_allowance_s', DEFAULT_TIME_ALLOWANCE_S, 'command_max_time_allowance_s', 1.0),
    'target_yaw_rad': ('angle_deg', 0.0, 'command_max_angle_deg', 180.0 / math.pi),
    'distance_m': ('distance_m', 0.0, 'command_max_distance_m', 1.0),
    'speed_mps': ('speed_mps', DEFAULT_DRIVE_SPEED_MPS, 'command_max_speed_mps', 1.0),
}
_POSITIVE_FIELDS = {'time_allowance_s', 'speed_mps'}


def _shown(number):
    """Up to two decimals, without trailing zeros: 400, 0.8, 6.28."""
    text = f'{round(number, 2):.2f}'.rstrip('0').rstrip('.')
    return '0' if text == '-0' else text


def refusal_message(command, params, limits, reason):
    """What a reason from check_command_params means for this command, for
    the person or AI that sent it: the field as they gave it, the value and
    the limit, in their units. None for other reasons."""
    kind, _, field = reason.partition(':')
    if field == 'params':
        return 'params must be an object'
    if field == 'command':
        return 'command must be spin or drive'
    if field not in _FIELDS:
        return None
    shown_field, default, setting, scale = _FIELDS[field]
    if kind == 'invalid_params':
        if field in _POSITIVE_FIELDS:
            return f'{shown_field} must be a number above 0'
        return f'{shown_field} must be a number'
    if kind != 'limit_exceeded':
        return None
    value = _as_finite_number((params or {}).get(field, default))
    limit = {
        'time_allowance_s': limits.max_time_allowance_s,
        'target_yaw_rad': limits.max_angle_deg,
        'distance_m': limits.max_distance_m,
        'speed_mps': limits.max_speed_mps,
    }[field]
    note = ' (the default when none is given)' if field not in (params or {}) else ''
    return f"{shown_field} {_shown(value * scale)}{note} is over this robot's limit of {_shown(limit)} ({setting})"
