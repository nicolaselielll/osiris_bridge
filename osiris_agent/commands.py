"""Robot motion commands: Nav2 spin and drive goals, gated by command_policy.py and the battery and dock checks."""

import threading
import time

from rclpy.action import ActionClient

from .command_policy import (
    DEFAULT_DRIVE_SPEED_MPS,
    DEFAULT_TIME_ALLOWANCE_S,
    check_command_params,
    refusal_message,
)

# Fixed enum of Nav2 actions send_command_request can trigger — deliberately
# not a generic "call any action with any payload" capability. Each entry:
# action_name = the ROS2 action server this sends goals to.
# module/cls   = lazily imported (nav2_msgs may not be installed everywhere).
# precondition_nodes = lifecycle-managed nodes that must be 'active' (per
#   _lifecycle_state_cache) beyond the action server itself existing.
#
# navigate_to_pose (absolute map-frame navigation) is deliberately not
# included — its precondition check would need more than a lifecycle-active
# check on /amcl, which only confirms AMCL is running, not that it's actually
# converged to a good estimate.
COMMAND_DEFS = {
    'spin': {
        'action_name': '/spin',
        'module': 'nav2_msgs.action',
        'cls_name': 'Spin',
        'precondition_nodes': [],
    },
    # Straight-line relative drive, robot-base-frame — same nav2_behaviors
    # family as spin, no localization dependency. It doesn't need AMCL to be
    # trustworthy, and it's guaranteed to be a straight line, not just *a*
    # planned path — both matter when a LiDAR wall fit is used as ground
    # truth for the robot's heading.
    'drive': {
        'action_name': '/drive_on_heading',
        'module': 'nav2_msgs.action',
        'cls_name': 'DriveOnHeading',
        'precondition_nodes': [],
    },
}

# How old a cached battery/dock reading is allowed to be before a send_command
# request treats it as unknown rather than trusting it (see
# _check_battery_and_dock_preconditions). Covers both "never received a
# message" and "the publisher went away" with the same fail-closed check —
# either way, no reading newer than this means no trustworthy current answer.
STATUS_STALE_AFTER_S = 10.0

# Safety-valve for the "one command at a time" slot: normally it's released by
# _on_command_goal_result, which only fires once the goal's result actually
# arrives — itself dependent on the goal cleanly reaching a terminal state on
# the ROS2 side. A behavior that never calls succeed()/abort()/is never
# properly cancelled leaves that callback permanently unfired, wedging every
# future send_command_request behind 'command_already_in_progress' until the
# agent process is restarted. This bounds that, and stays generous because a
# slow command is normal, not a hang. It's enforced agent-side because the
# gateway's own wait for the result is separate and per-request, and that
# wait timing out doesn't clear this slot either.
ACTIVE_COMMAND_TIMEOUT_S = 120.0

# Actions whose goals move the robot. A drive or spin sent while a goal on one of
# them is running would command the base at the same time, so it is refused.
# Matched on the last name segment, so a namespaced Nav2 counts too.
MOTION_ACTIONS = {
    'navigate_to_pose', 'navigate_through_poses', 'follow_waypoints', 'follow_gps_waypoints',
    'follow_path', 'spin', 'backup', 'drive_on_heading', 'assisted_teleop', 'dock_robot', 'undock_robot',
}
ACTIVE_GOAL_STATUSES = (1, 2, 3)  # ACCEPTED, EXECUTING, CANCELING


class CommandMixin:

    def _resolve_command_action_name(self, command: str) -> str | None:
        """The command's action server in the current graph: the one in this
        agent's own namespace, else the only one there is (a Nav2 under a
        namespace such as /robot1), else the root name. None when several
        namespaces have one and none is this agent's: which robot to move is
        then not something to guess."""
        command_def = COMMAND_DEFS[command]
        default = command_def['action_name']
        base = default.rsplit('/', 1)[-1]
        goal_type = f"{command_def['module'].replace('.', '/')}/{command_def['cls_name']}_Goal"
        candidates = []
        for action in sorted(self._active_actions):
            if action.rsplit('/', 1)[-1] != base:
                continue
            info = self._action_type_cache.get(action)
            if info and info.get('goal_type') != goal_type:
                continue
            candidates.append(action)
        own = f"{self.get_namespace().rstrip('/')}/{base}"
        if own in candidates:
            return own
        if len(candidates) == 1:
            return candidates[0]
        if not candidates:
            return default
        return None

    def _get_command_action_client(self, command: str, action_name: str):
        """Lazily imports the action type and creates (once per action name) the
        ActionClient for this command. Returns None if the type isn't importable
        (package not installed) — a permanent condition for this process, not
        worth retrying on every request."""
        key = (command, action_name)
        if key in self._command_action_clients:
            return self._command_action_clients[key]

        command_def = COMMAND_DEFS[command]
        try:
            module = __import__(command_def['module'], fromlist=[command_def['cls_name']])
            msg_cls = getattr(module, command_def['cls_name'])
        except ImportError as e:
            self.get_logger().warning(f'[command] {command} unavailable, import failed: {e}')
            self._command_action_clients[key] = None
            return None

        client = ActionClient(self, msg_cls, action_name)
        self._command_action_clients[key] = client
        return client

    def _active_motion_action(self) -> str | None:
        """An action in the graph with a goal that is moving the robot right now,
        or None. Goal states come from the action status topics the agent
        already follows (_on_action_status)."""
        for action, goals in list(self._action_goal_states.items()):
            if action not in self._active_actions or action.rsplit('/', 1)[-1] not in MOTION_ACTIONS:
                continue
            if any(status in ACTIVE_GOAL_STATUSES for status in goals.values()):
                return action
        return None

    @staticmethod
    def _seconds_to_duration(seconds: float):
        from builtin_interfaces.msg import Duration
        d = Duration()
        d.sec = int(seconds)
        d.nanosec = int(round((seconds - d.sec) * 1e9))
        return d

    def _apply_shared_goal_fields(self, goal, command: str, time_allowance_s: float, disable_collision_checks: bool):
        # time_allowance and disable_collision_checks are shared by both
        # Spin.action and DriveOnHeading.action; each action's own fields
        # (target_yaw, target/speed) are set by the callers below. An unset
        # time_allowance is a zero Duration,
        # which nav2_behaviors treats as unbounded, so a behavior stuck never
        # reaching its stop condition would have no time-based backstop at
        # all. DEFAULT_TIME_ALLOWANCE_S gives every command a bounded
        # worst-case runtime by default while staying overridable; still
        # comfortably under the gateway's own 120s wait for a terminal
        # result, so Nav2 gets a chance to self-abort before that gives up
        # waiting.
        goal.time_allowance = self._seconds_to_duration(time_allowance_s)

        # disable_collision_checks was added to Spin.action/DriveOnHeading.
        # action after Humble (Humble's goal only has target_yaw/target/speed/
        # time_allowance), and Humble is supported. Older Nav2 installs have
        # no way to disable the check at all, so collision checking stays on
        # regardless of what's asked; hasattr avoids the AttributeError
        # ROS2's slotted generated message classes raise for an unknown
        # field. An explicit request that can't be honored is logged rather
        # than silently dropped, but still proceeds with checking ON (the
        # safe fallback, and the only option this install actually has).
        if hasattr(goal, 'disable_collision_checks'):
            goal.disable_collision_checks = disable_collision_checks
        elif disable_collision_checks:
            self.get_logger().warning(
                f'[command] disable_collision_checks was requested for {command} but this '
                f"Nav2 install's {type(goal).__name__} has no such field (added after Humble) "
                f'— proceeding with collision checking ON.'
            )

    def _build_command_goal(self, command: str, params: dict, msg_cls):
        time_allowance_s = float(params.get('time_allowance_s', DEFAULT_TIME_ALLOWANCE_S))
        disable_collision_checks = bool(params.get('disable_collision_checks', False))

        if command == 'spin':
            goal = msg_cls.Goal()
            goal.target_yaw = float(params.get('target_yaw_rad', 0.0))
            self._apply_shared_goal_fields(goal, command, time_allowance_s, disable_collision_checks)
            return goal

        if command == 'drive':
            goal = msg_cls.Goal()
            # target is a relative point in the robot's own base frame — x is
            # straight ahead (negative = backward), y/z stay 0 for a pure
            # straight-line drive.
            distance = float(params.get('distance_m', 0.0))
            goal.target.x = distance
            goal.target.y = 0.0
            goal.target.z = 0.0
            # DriveOnHeading rejects a distance and a speed with opposite signs
            # ("Initial checks failed"), so a backward drive needs a negative speed.
            speed = abs(float(params.get('speed_mps', DEFAULT_DRIVE_SPEED_MPS)))
            goal.speed = -speed if distance < 0 else speed
            self._apply_shared_goal_fields(goal, command, time_allowance_s, disable_collision_checks)
            return goal

        raise ValueError(f'no goal builder for command {command}')

    def _check_battery_and_dock_preconditions(self) -> str | None:
        """Returns a precondition_failed:* reason string if it isn't safe to
        move right now, else None. Each check is independently opt-in via its
        own toggle (battery_check_enabled / dock_check_enabled) — a robot
        that hasn't turned one on just doesn't get gated by it at all. Once
        enabled, this is universal and unconditional: it applies to every
        command in COMMAND_DEFS the same way, with no way for a caller to
        pass a flag to skip it (unlike disable_collision_checks, which is a
        deliberate per-call opt-out of a different check).

        Fails closed on missing/stale data, not just on an actual bad
        reading: no cached value at all, or one older than
        STATUS_STALE_AFTER_S, is treated the same as an unsafe reading
        ('_unknown' reasons below) rather than silently letting the command
        through. This covers both "never received a message on this topic"
        (e.g. a latched publisher whose earlier message our subscription's
        QoS doesn't receive — see STATUS_QOS in qos.py) and "the
        publisher died" with the same mechanism.
        """
        now = time.time()

        if self._battery_check_enabled:
            if self._last_battery_state is None or self._last_battery_state_time is None:
                return 'precondition_failed:battery_unknown'
            if now - self._last_battery_state_time > STATUS_STALE_AFTER_S:
                return 'precondition_failed:battery_unknown'
            percent = self._last_battery_state.get('percent')
            if percent is None:
                return 'precondition_failed:battery_unknown'
            if percent < self._battery_min_percent:
                return 'precondition_failed:battery_low'

        if self._dock_check_enabled:
            if self._last_dock_status is None or self._last_dock_status_time is None:
                return 'precondition_failed:dock_status_unknown'
            if now - self._last_dock_status_time > STATUS_STALE_AFTER_S:
                return 'precondition_failed:dock_status_unknown'
            if self._last_dock_status.get('is_docked'):
                return 'precondition_failed:docked'

        return None

    def _handle_send_command_request(self, data: dict):
        request_id = data.get('request_id', '')
        command = data.get('command')
        params = data.get('params') or {}

        command_def = COMMAND_DEFS.get(command)
        if not command_def:
            self._send_command_rejected(request_id, 'unknown_command')
            return

        # Cancelling a running command (_handle_cancel_command_request) is
        # deliberately not gated: stopping the robot must always work.
        if not self._commands_enabled:
            self._send_command_rejected(request_id, 'commands_disabled')
            return

        limit_reason = check_command_params(command, params, self._command_limits)
        if limit_reason:
            self._send_command_rejected(
                request_id, limit_reason,
                refusal_message(command, params, self._command_limits, limit_reason),
            )
            return

        if not self._goals_enabled:
            self._send_command_rejected(request_id, 'goals_disabled')
            return

        # Only one AI/user-commanded goal in flight at a time — claim the
        # slot immediately so two overlapping requests can't both pass this
        # check before either resolves. Cleared on reject, on accept-failure,
        # and once the accepted goal reaches a terminal state (_on_command_goal_result).
        with self._active_command_lock:
            if self._active_command_pending or self._active_command_goal_handle is not None:
                self._send_command_rejected(request_id, 'command_already_in_progress')
                return
            self._active_command_pending = True

        def _release_pending():
            with self._active_command_lock:
                self._active_command_pending = False

        # Another goal is already moving the robot (a Nav2 navigation, a dock,
        # a spin someone else started): two sources would command the base.
        # Checked after the slot, so this agent's own running command is
        # reported as command_already_in_progress.
        busy_action = self._active_motion_action()
        if busy_action:
            _release_pending()
            self._send_command_rejected(request_id, f'motion_in_progress:{busy_action}')
            return

        action_name = self._resolve_command_action_name(command)
        if action_name is None:
            _release_pending()
            self._send_command_rejected(request_id, 'action_server_ambiguous')
            return
        client = self._get_command_action_client(command, action_name)
        if client is None:
            _release_pending()
            self._send_command_rejected(request_id, 'action_type_unavailable')
            return

        # Non-blocking — deliberately not wait_for_server(), which blocks the
        # asyncio loop thread this runs on for up to its whole timeout and
        # would freeze all other WS traffic while waiting. If the server
        # isn't known-ready right now, fail fast rather than stall.
        if not client.server_is_ready():
            _release_pending()
            self._send_command_rejected(request_id, 'action_server_not_available')
            return

        # Precondition: for a command that lists precondition_nodes, the
        # action server existing isn't enough on its own; those
        # lifecycle-managed nodes must be active too (spin and drive list
        # none). Reuses _lifecycle_state_cache, which the graph watcher already keeps
        # live via transition-event subscriptions (see _on_lifecycle_transition),
        # not a one-off snapshot.
        for node_fqn in command_def['precondition_nodes']:
            state = self._lifecycle_state_cache.get(node_fqn)
            if state != 'active':
                _release_pending()
                self._send_command_rejected(request_id, f'precondition_failed:{node_fqn}_not_active')
                return

        # Battery/dock safety check — universal across every command in
        # COMMAND_DEFS, not per-command opt-in like precondition_nodes above,
        # and not something the caller can pass a flag to skip. See
        # _check_battery_and_dock_preconditions for what each check requires.
        safety_reason = self._check_battery_and_dock_preconditions()
        if safety_reason:
            _release_pending()
            self._send_command_rejected(request_id, safety_reason)
            return

        try:
            module = __import__(command_def['module'], fromlist=[command_def['cls_name']])
            msg_cls = getattr(module, command_def['cls_name'])
            goal_msg = self._build_command_goal(command, params, msg_cls)
        except Exception as e:
            _release_pending()
            self._send_command_rejected(request_id, f'invalid_params:{e}')
            return

        send_future = client.send_goal_async(goal_msg)
        send_future.add_done_callback(
            lambda fut, rid=request_id, cmd=command: self._on_command_send_response(fut, rid, cmd)
        )

    def _on_command_send_response(self, future, request_id: str, command: str):
        try:
            goal_handle = future.result()
        except Exception as e:
            with self._active_command_lock:
                self._active_command_pending = False
            self._send_command_rejected(request_id, f'send_failed:{e}')
            return

        if not goal_handle.accepted:
            with self._active_command_lock:
                self._active_command_pending = False
            self._send_command_rejected(request_id, 'goal_rejected_by_server')
            return

        with self._active_command_lock:
            self._active_command_pending = False
            self._active_command_goal_handle = goal_handle
            timer = threading.Timer(
                ACTIVE_COMMAND_TIMEOUT_S,
                lambda gh=goal_handle: self._force_release_active_command(gh),
            )
            timer.daemon = True
            self._active_command_timeout_timer = timer
            timer.start()

        self._enqueue({
            'type': 'command_accepted',
            'request_id': request_id,
            'command': command,
            'goal_id': bytes(goal_handle.goal_id.uuid).hex(),
            'timestamp': time.time(),
        })

        # Progress/outcome is already reported via the existing generic
        # goal_event pipeline (_on_action_status monitors every action in the
        # graph, not just ones this code sent). This callback exists only to
        # release the "one command at a time" slot once this goal is done —
        # the normal path. _force_release_active_command (via the timer
        # started above) is the fallback for a goal that never gets here at
        # all, e.g. a behavior that never reaches a terminal state on its own
        # and doesn't cleanly terminate after a cancel either.
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(lambda fut, gh=goal_handle: self._on_command_goal_result(gh))

    def _on_command_goal_result(self, goal_handle):
        with self._active_command_lock:
            if self._active_command_goal_handle is goal_handle:
                self._active_command_goal_handle = None
                if self._active_command_timeout_timer is not None:
                    self._active_command_timeout_timer.cancel()
                    self._active_command_timeout_timer = None

    def _force_release_active_command(self, goal_handle):
        """Fires from the timeout timer, on whatever thread threading.Timer
        uses — not the ROS executor thread, same as the other threading-based
        timers in the agent (_trigger_graph_poll). Only acts if this is still
        the SAME goal that's still stuck; a goal that finished normally
        already cleared both the handle and this timer together in
        _on_command_goal_result, so a late/stale timer firing is a no-op."""
        with self._active_command_lock:
            if self._active_command_goal_handle is not goal_handle:
                return
            self._active_command_goal_handle = None
            self._active_command_timeout_timer = None
        self.get_logger().warning(
            f'[command] goal {bytes(goal_handle.goal_id.uuid).hex()} never reached a terminal '
            f'state within {ACTIVE_COMMAND_TIMEOUT_S}s — force-releasing the command slot so '
            f'future commands aren\'t blocked. The underlying ROS goal may still be running; '
            f'this only unblocks send_command_request, it does not cancel anything.'
        )

    def _send_command_rejected(self, request_id: str, reason: str, message: str | None = None):
        payload = {
            'type': 'command_rejected',
            'request_id': request_id,
            'reason': reason,
            'timestamp': time.time(),
        }
        # What the reason means for this command, for the person or AI that
        # sent it. Older gateways leave it out.
        if message:
            payload['message'] = message
        self._enqueue(payload)

    def _handle_cancel_command_request(self, data: dict):
        """Manual stop — bypasses the AI entirely by design. Cancels whatever
        command is currently active/in-flight, regardless of what requested
        it or what conversation (if any) triggered it."""
        request_id = data.get('request_id', '')
        with self._active_command_lock:
            goal_handle = self._active_command_goal_handle
        if goal_handle is None:
            self._enqueue({
                'type': 'cancel_failed',
                'request_id': request_id,
                'reason': 'no_active_command',
                'timestamp': time.time(),
            })
            return

        cancel_future = goal_handle.cancel_goal_async()

        def _on_cancel_done(fut, rid=request_id):
            self._enqueue({
                'type': 'command_cancelled',
                'request_id': rid,
                'timestamp': time.time(),
            })

        cancel_future.add_done_callback(_on_cancel_done)

    def _on_dock_status(self, msg) -> None:
        """Cache the latest opennav_docking_msgs/DockStatus message for the
        send_command docked-status precondition check."""
        try:
            self._last_dock_status = {
                'is_docked':   bool(msg.is_docked),
                'is_charging': bool(msg.is_charging),
            }
            self._last_dock_status_time = time.time()
        except Exception:
            pass
