"""
Collector for ros2_control state: controllers and hardware components.

Uses controller_manager services (ListControllers, ListHardwareComponents) to
gather controller and hardware component state.  Polled every poll_interval by
the agent's own timer, and from the graph check at most once per
poll_interval, with its own change-detection guard.
"""

import time
from datetime import datetime, timezone

# A list request still unanswered after this long is given up on, so a
# controller_manager that stops answering can't block polling for good.
REQUEST_TIMEOUT_S = 10.0


class Ros2ControlCollector:
    """Polls controller_manager services for controller & hardware state."""

    # controller_manager_msgs may not be installed — import is deferred to
    # first use and the collector degrades gracefully if the package is absent.
    _cm_msgs_available = None  # None = not checked yet

    def __init__(self, node, event_callback, logger, poll_interval: float = 2.0):
        self._node = node
        self._event_callback = event_callback
        self._logger = logger

        # Change-detection caches (serialisable dicts, compared by equality)
        self._last_sent_controllers = None
        self._last_sent_hardware = None

        # Per-controller previous state, used to track last_switch_time
        self._controller_prev_states: dict[str, str] = {}
        self._controller_switch_times: dict[str, str] = {}

        # Reusable service clients (created once, kept alive)
        self._list_controllers_client = None
        self._list_hardware_client = None

        # The list requests still waiting for an answer, as (future, sent_at):
        # a controller_manager that's slow to answer gets no second request
        # meanwhile, so requests can't pile up.
        self._controllers_request = None
        self._hardware_request = None

        # Rate-limiting: poll at most once per poll_interval seconds, unless
        # forced (the agent's poll timer forces, the graph check doesn't)
        self._last_poll_time = 0.0
        self._poll_interval = poll_interval

        # Set by destroy(), so a poll racing with it doesn't recreate clients
        self._destroyed = False

    # ------------------------------------------------------------------
    # Public API – called from _check_graph_changes and the poll timer
    # ------------------------------------------------------------------

    def poll(self, force: bool = False):
        """Check controller_manager state and emit events on change."""
        if self._destroyed:
            return
        now = time.time()
        if not force and now - self._last_poll_time < self._poll_interval:
            return
        self._last_poll_time = now

        if not self._ensure_cm_msgs():
            return  # controller_manager_msgs not installed

        if not self._controller_manager_available():
            # controller_manager has gone away — clear cached state so that
            # when it comes back the data is re-sent, and emit empty lists if
            # we previously had data. A request it never answered won't be
            # answered now, so it no longer holds back the next one.
            self._controllers_request = self._hardware_request = None
            if self._last_sent_controllers is not None:
                self._last_sent_controllers = None
                self._controller_prev_states.clear()
                self._controller_switch_times.clear()
                self._event_callback({
                    'type': 'controllers',
                    'data': [],
                    'timestamp': time.time(),
                })
            if self._last_sent_hardware is not None:
                self._last_sent_hardware = None
                self._event_callback({
                    'type': 'hardware',
                    'data': [],
                    'timestamp': time.time(),
                })
            return

        self._poll_controllers()
        self._poll_hardware()

    def get_controllers_snapshot(self):
        """Return last known controllers state (for initial_state)."""
        return self._last_sent_controllers

    def get_hardware_snapshot(self):
        """Return last known hardware state (for initial_state)."""
        return self._last_sent_hardware

    # ------------------------------------------------------------------
    # Internals
    # ------------------------------------------------------------------

    def _ensure_cm_msgs(self):
        """Lazily check whether controller_manager_msgs is importable."""
        if Ros2ControlCollector._cm_msgs_available is not None:
            return Ros2ControlCollector._cm_msgs_available

        try:
            from controller_manager_msgs.srv import (  # noqa: F401
                ListControllers,
                ListHardwareComponents,
            )
            Ros2ControlCollector._cm_msgs_available = True
        except ImportError:
            self._logger.info(
                "controller_manager_msgs not found – ros2_control panels disabled"
            )
            Ros2ControlCollector._cm_msgs_available = False

        return Ros2ControlCollector._cm_msgs_available

    def _controller_manager_available(self):
        """Return True if /controller_manager is among the live nodes."""
        node_names = self._node.get_node_names()
        return any(
            n == '/controller_manager' or n == 'controller_manager'
            for n in node_names
        )

    @staticmethod
    def _still_waiting(client, request):
        """True while the previous request is unanswered and younger than
        REQUEST_TIMEOUT_S. An older one is dropped, so a late answer to it
        is ignored."""
        if request is None:
            return False
        future, sent_at = request
        if future.done():
            return False
        if time.time() - sent_at < REQUEST_TIMEOUT_S:
            return True
        client.remove_pending_request(future)
        return False

    # -- Controllers ---------------------------------------------------

    def _poll_controllers(self):
        from controller_manager_msgs.srv import ListControllers

        if self._list_controllers_client is None:
            self._list_controllers_client = self._node.create_client(
                ListControllers,
                '/controller_manager/list_controllers',
            )

        if not self._list_controllers_client.service_is_ready():
            return
        if self._still_waiting(self._list_controllers_client, self._controllers_request):
            return

        request = ListControllers.Request()
        future = self._list_controllers_client.call_async(request)
        self._controllers_request = (future, time.time())
        future.add_done_callback(self._on_controllers_response)

    def _parse_controllers(self, response):
        """Convert ListControllers.Response → list[dict]."""
        now_iso = datetime.now(timezone.utc).isoformat()
        result = []

        for c in response.controller:
            name = c.name
            state = c.state  # e.g. 'active', 'inactive', 'configured'

            # Track state transitions for last_switch_time
            prev = self._controller_prev_states.get(name)
            if prev is not None and prev != state:
                self._controller_switch_times[name] = now_iso
            self._controller_prev_states[name] = state

            claimed = []
            if hasattr(c, 'claimed_interfaces'):
                claimed = list(c.claimed_interfaces)

            required_cmd = []
            if hasattr(c, 'required_command_interfaces'):
                required_cmd = list(c.required_command_interfaces)

            required_state = []
            if hasattr(c, 'required_state_interfaces'):
                required_state = list(c.required_state_interfaces)

            # Reference interfaces (ros2_control >= Humble patch)
            reference_interfaces = []
            if hasattr(c, 'reference_interfaces'):
                reference_interfaces = list(c.reference_interfaces)

            result.append({
                'name': name,
                'type': c.type if hasattr(c, 'type') else 'unknown',
                'state': state,
                'claimed_interfaces': claimed,
                'required_command_interfaces': required_cmd,
                'required_state_interfaces': required_state,
                'reference_interfaces': reference_interfaces,
                'last_switch_time': self._controller_switch_times.get(name),
            })

        # Sort for stable comparison
        result.sort(key=lambda x: x['name'])
        return result

    def _on_controllers_response(self, future):
        try:
            response = future.result()
        except Exception:
            return
        if response is None:
            return
        controllers = self._parse_controllers(response)
        if controllers != self._last_sent_controllers:
            self._last_sent_controllers = controllers
            self._event_callback({
                'type': 'controllers',
                'data': controllers,
                'timestamp': time.time(),
            })

    # -- Hardware ------------------------------------------------------

    def _poll_hardware(self):
        from controller_manager_msgs.srv import ListHardwareComponents

        if self._list_hardware_client is None:
            self._list_hardware_client = self._node.create_client(
                ListHardwareComponents,
                '/controller_manager/list_hardware_components',
            )

        if not self._list_hardware_client.service_is_ready():
            return
        if self._still_waiting(self._list_hardware_client, self._hardware_request):
            return

        request = ListHardwareComponents.Request()
        future = self._list_hardware_client.call_async(request)
        self._hardware_request = (future, time.time())
        future.add_done_callback(self._on_hardware_response)

    def _parse_hardware(self, response):
        """Convert ListHardwareComponents.Response → list[dict]."""
        result = []

        for comp in response.component:
            cmd_interfaces = []
            if hasattr(comp, 'command_interfaces'):
                for iface in comp.command_interfaces:
                    entry = {'name': iface.name}
                    if hasattr(iface, 'is_available'):
                        entry['is_available'] = iface.is_available
                    if hasattr(iface, 'is_claimed'):
                        entry['is_claimed'] = iface.is_claimed
                    cmd_interfaces.append(entry)

            state_interfaces = []
            if hasattr(comp, 'state_interfaces'):
                for iface in comp.state_interfaces:
                    entry = {'name': iface.name}
                    if hasattr(iface, 'is_available'):
                        entry['is_available'] = iface.is_available
                    state_interfaces.append(entry)

            state_label = 'unknown'
            state_id = -1
            if hasattr(comp, 'state'):
                if hasattr(comp.state, 'label'):
                    state_label = comp.state.label
                if hasattr(comp.state, 'id'):
                    state_id = comp.state.id

            result.append({
                'name': comp.name,
                'type': comp.type if hasattr(comp, 'type') else 'unknown',
                'class_type': comp.class_type if hasattr(comp, 'class_type') else 'unknown',
                'state': {
                    'label': state_label,
                    'id': state_id,
                },
                'command_interfaces': cmd_interfaces,
                'state_interfaces': state_interfaces,
            })

        result.sort(key=lambda x: x['name'])
        return result

    def _on_hardware_response(self, future):
        try:
            response = future.result()
        except Exception:
            return
        if response is None:
            return
        hardware = self._parse_hardware(response)
        if hardware != self._last_sent_hardware:
            self._last_sent_hardware = hardware
            self._event_callback({
                'type': 'hardware',
                'data': hardware,
                'timestamp': time.time(),
            })

    # -- Cleanup -------------------------------------------------------

    def destroy(self):
        """Destroy any open service clients."""
        self._destroyed = True
        if self._list_controllers_client is not None:
            self._node.destroy_client(self._list_controllers_client)
            self._list_controllers_client = None
        if self._list_hardware_client is not None:
            self._node.destroy_client(self._list_hardware_client)
            self._list_hardware_client = None
            