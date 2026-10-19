"""Lifecycle monitoring: tracks managed nodes' states through transition events and get_state."""

import time

from rclpy.qos import QoSProfile


class LifecycleMixin:

    def _subscribe_lifecycle_topic(self, topic: str):
        """Subscribe to a /<node>/transition_event topic."""
        if topic in self._lifecycle_subs:
            return
        try:
            from lifecycle_msgs.msg import TransitionEvent
            node_fqn = topic[:-len('/transition_event')]
            sub = self.create_subscription(
                TransitionEvent,
                topic,
                lambda msg, n=node_fqn: self._on_lifecycle_transition(msg, n),
                QoSProfile(depth=10),
            )
            self._lifecycle_subs[topic] = sub
            self.get_logger().info(f'[lifecycle] subscribed to {topic}')
        except Exception as e:
            self.get_logger().debug(f'[lifecycle] could not subscribe to {topic}: {e}')

    def _fetch_lifecycle_state_async(self, node_fqn: str):
        """Query /<node>/get_state to populate _lifecycle_state_cache."""
        if node_fqn in self._lifecycle_state_cache:
            return
        if node_fqn in self._pending_lifecycle_fetches:
            return
        try:
            from lifecycle_msgs.srv import GetState
        except ImportError:
            return
        client = self.create_client(GetState, f'{node_fqn}/get_state')
        if not client.service_is_ready():
            self.destroy_client(client)
            return
        self._pending_lifecycle_fetches.add(node_fqn)
        future = client.call_async(GetState.Request())

        def _on_get_state(fut):
            self._pending_lifecycle_fetches.discard(node_fqn)
            self.destroy_client(client)
            try:
                resp = fut.result()
                if resp is not None:
                    self._lifecycle_state_cache[node_fqn] = resp.current_state.label
                    self._graph_dirty = True
                    self.get_logger().debug(
                        f'[lifecycle] initial state for {node_fqn}: {resp.current_state.label}'
                    )
            except Exception as e:
                self.get_logger().debug(f'[lifecycle] get_state failed for {node_fqn}: {e}')

        future.add_done_callback(_on_get_state)

    def _on_lifecycle_transition(self, msg, node_fqn: str):
        self._lifecycle_state_cache[node_fqn] = msg.goal_state.label
        self.get_logger().info(
            f'[lifecycle] {node_fqn}: {msg.start_state.label} → {msg.goal_state.label} '
            f'(transition: {msg.transition.label})'
        )
        self._enqueue({
            'type': 'lifecycle_event',
            'node': node_fqn,
            'transition': msg.transition.label,
            'from_state': msg.start_state.label,
            'to_state': msg.goal_state.label,
            'timestamp': time.time(),
        })
        self._graph_dirty = True
        self._trigger_graph_poll()
