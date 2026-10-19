"""Action monitoring: resolves action types and forwards goal status changes and feedback."""

import time

from rclpy.qos import QoSProfile, qos_profile_action_status_default
from rosidl_runtime_py import message_to_ordereddict
from rosidl_runtime_py.utilities import get_message

ACTION_FEEDBACK_MIN_INTERVAL = 0.2  # seconds between forwarded feedback messages per action (5 Hz cap)

# action_msgs/GoalStatus: SUCCEEDED, CANCELED, ABORTED.
TERMINAL_GOAL_STATUSES = (4, 5, 6)


class ActionMonitorMixin:

    def _fetch_action_types(self, action_name: str, topic_type_map: dict) -> bool:
        """Resolve and cache goal/result/feedback types for an action server.

        Looks up the [action]/_action/feedback topic type, strips the
        _FeedbackMessage suffix to derive the base action type, then imports
        the _Goal / _Result / _Feedback message classes to introspect fields.

        Returns True if types were resolved and enqueued, False if not yet
        available (e.g. the feedback topic hasn't appeared in DDS yet).
        """
        if action_name in self._action_type_cache:
            return self._action_type_cache[action_name] is not None

        feedback_topic = f'{action_name}/_action/feedback'
        types_list = topic_type_map.get(feedback_topic)
        if not types_list:
            return False

        feedback_msg_type = types_list[0]
        if not feedback_msg_type.endswith('_FeedbackMessage'):
            return False

        base_type = feedback_msg_type[:-len('_FeedbackMessage')]

        try:
            # base_type is e.g. 'nav2_msgs/action/NavigateToPose'
            # Action sub-types (Goal/Result/Feedback) are nested on the action
            # class itself — get_message() only handles message types, not actions.
            import importlib
            pkg, _, action_name_part = base_type.split('/', 2)
            # action_name_part may be 'action/NavigateToPose' — take just the class name
            class_name = action_name_part.split('/')[-1]
            action_mod = importlib.import_module(f'{pkg}.action')
            action_cls = getattr(action_mod, class_name)
            # Use get_message() for the _FeedbackMessage type — it IS a standalone message
            # type (unlike _Goal/_Result/_Feedback which are nested). The type string
            # comes directly from the topic registry so it's always correct.
            feedback_msg_cls = get_message(feedback_msg_type)

            goal_cls     = action_cls.Goal
            result_cls   = action_cls.Result
            feedback_cls = action_cls.Feedback

            def _fields(cls):
                try:
                    return dict(cls.get_fields_and_field_types())
                except Exception:
                    return {}

            type_info = {
                'goal_type':       f'{base_type}_Goal',
                'result_type':     f'{base_type}_Result',
                'feedback_type':   f'{base_type}_Feedback',
                'goal_fields':     _fields(goal_cls),
                'result_fields':   _fields(result_cls),
                'feedback_fields': _fields(feedback_cls),
                # Wire message type (not the same string as feedback_type
                # above — that's the nested class name, this is the
                # standalone _FeedbackMessage actually subscribed to) kept
                # so a later goals_enabled toggle-on can catch up the
                # feedback subscription without redoing type resolution —
                # see _catch_up_action_feedback.
                '_feedback_msg_type': feedback_msg_type,
            }
            self._action_type_cache[action_name] = type_info
            self.get_logger().info(f'[actions] resolved types for {action_name}: {base_type}')
            self._graph_dirty = True
            if self._goals_enabled:
                self._subscribe_action_feedback(action_name, feedback_msg_cls)
            return True
        except Exception as e:
            self.get_logger().warning(f'[actions] failed to resolve types for {action_name}: {e}')
            self._action_type_cache[action_name] = None  # mark failed — avoid retry spam
            return False

    def _subscribe_action_status(self, action_name: str):
        if action_name in self._action_status_subs:
            return
        try:
            from action_msgs.msg import GoalStatusArray
            # Action status publishers use RELIABLE + TRANSIENT_LOCAL
            # (rcl_action_qos_profile_status_default), not the plain default
            # (VOLATILE) QoSProfile(depth=10) used elsewhere — a durability
            # mismatch here means a subscription created after a goal already
            # reached a terminal state (agent restart, resubscribe, or any
            # gap in this specific subscription's lifetime) would never
            # receive that goal's actual last status. qos_profile_action_
            # status_default is rclpy's own copy of that exact profile.
            sub = self.create_subscription(
                GoalStatusArray,
                f'{action_name}/_action/status',
                lambda msg, a=action_name: self._on_action_status(msg, a),
                qos_profile_action_status_default,
            )
            self._action_status_subs[action_name] = sub
            self._action_status_unread.add(action_name)
            self.get_logger().info(f'[actions] subscribed to status for {action_name}')
        except Exception as e:
            self.get_logger().warning(f'[actions] failed to subscribe to status for {action_name}: {e}')

    def _subscribe_action_feedback(self, action_name: str, feedback_msg_cls):
        if action_name in self._action_feedback_subs:
            return
        try:
            sub = self.create_subscription(
                feedback_msg_cls,
                f'{action_name}/_action/feedback',
                lambda msg, a=action_name: self._on_action_feedback(msg, a),
                QoSProfile(depth=10),
            )
            self._action_feedback_subs[action_name] = sub
            self.get_logger().info(f'[actions] subscribed to feedback for {action_name}')
        except Exception as e:
            self.get_logger().warning(f'[actions] failed to subscribe to feedback for {action_name}: {e}')

    def _catch_up_action_feedback(self, action_name: str):
        """Subscribes to feedback for an action whose type was already
        resolved (cached) while goals_enabled was False. _fetch_action_types
        early-returns once an action is cached, so simply re-calling it here
        would silently no-op instead of subscribing — this reuses the
        cached _feedback_msg_type instead of redoing type resolution."""
        type_info = self._action_type_cache.get(action_name)
        if not type_info:
            return
        try:
            feedback_msg_cls = get_message(type_info['_feedback_msg_type'])
            self._subscribe_action_feedback(action_name, feedback_msg_cls)
        except Exception as e:
            self.get_logger().warning(f'[actions] failed to catch up feedback subscription for {action_name}: {e}')

    def _unsubscribe_action_monitoring(self, action_name: str):
        sub = self._action_status_subs.pop(action_name, None)
        if sub:
            self.destroy_subscription(sub)
        sub = self._action_feedback_subs.pop(action_name, None)
        if sub:
            self.destroy_subscription(sub)
        self._action_goal_states.pop(action_name, None)
        self._action_feedback_throttle.pop(action_name, None)
        self._action_status_unread.discard(action_name)

    def _on_action_status(self, msg, action_name: str):
        # A new subscription's first message is the status the action server
        # last published, kept for late joiners: after an agent restart it
        # still lists the action's last goals, however long ago they ended.
        # A goal already finished in it ended at a time this message doesn't
        # tell, so it is marked replayed rather than taken for a goal that
        # just ended.
        first = action_name in self._action_status_unread
        self._action_status_unread.discard(action_name)
        prev = self._action_goal_states.get(action_name, {})
        current = {bytes(s.goal_info.goal_id.uuid).hex(): s.status for s in msg.status_list}
        for uuid_hex, status in current.items():
            if uuid_hex not in prev or prev[uuid_hex] != status:
                event = {
                    'type': 'goal_event',
                    'action': action_name,
                    'goal_id': uuid_hex,
                    'status': status,
                    'timestamp': time.time(),
                }
                if first and status in TERMINAL_GOAL_STATUSES:
                    event['replayed'] = True
                self._enqueue(event)
        self._action_goal_states[action_name] = current

    def _on_action_feedback(self, msg, action_name: str):
        now = time.time()
        if now - self._action_feedback_throttle.get(action_name, 0.0) < ACTION_FEEDBACK_MIN_INTERVAL:
            return
        self._action_feedback_throttle[action_name] = now
        try:
            feedback_data = message_to_ordereddict(msg.feedback)
        except Exception as e:
            self.get_logger().warning(f'[actions] feedback serialization failed for {action_name}: {e}')
            return
        self._enqueue({
            'type': 'action_feedback',
            'action': action_name,
            'goal_id': bytes(msg.goal_id.uuid).hex(),
            'feedback': feedback_data,
            'timestamp': now,
        })
