"""Node parameters: fetches every node's parameters and keeps them current from /parameter_events."""

from rcl_interfaces.msg import ParameterEvent
from rcl_interfaces.srv import GetParameters, ListParameters
from rclpy.parameter import parameter_value_to_python

# Marks a parameter deleted in a set of recorded changes.
_DELETED = object()


def _with_changes(params: dict, changes: dict) -> dict:
    result = dict(params)
    for name, value in changes.items():
        if value is _DELETED:
            result.pop(name, None)
        else:
            result[name] = value
    return result


class NodeParametersMixin:

    def _on_parameter_event(self, msg: ParameterEvent):
        """React to parameter changes published by any node on /parameter_events."""
        if not self._params_enabled:
            return
        fqn = msg.node
        changes = {}
        for param in list(msg.new_parameters) + list(msg.changed_parameters):
            try:
                changes[param.name] = parameter_value_to_python(param.value)
            except Exception:
                pass
        for param in msg.deleted_parameters:
            changes[param.name] = _DELETED
        # A fetch in flight answers with what the node had when it was asked,
        # so changes made meanwhile are replayed on top of its answer.
        if fqn in self._param_fetch_changes:
            self._param_fetch_changes[fqn].update(changes)
        # A node is tracked once it is active or has been fetched: the poll that
        # finds a node fetches it before marking it active, and the fetch can
        # finish, and the node change more parameters, within that same poll.
        if fqn not in self._active_nodes and fqn not in self._node_parameter_cache:
            return
        cache = _with_changes(self._node_parameter_cache.get(fqn) or {}, changes)
        if cache != self._node_parameter_cache.get(fqn):
            self._node_parameter_cache[fqn] = cache
            self.get_logger().debug(f'[params] updated {len(cache)} params for {fqn} via /parameter_events')
            self._trigger_graph_poll()

    def _fetch_node_parameters_async(self, fqn: str):
        """Fetch parameters for *fqn* without blocking the executor.

        Creates service clients, fires async calls, and stores results in
        _node_parameter_cache when callbacks fire.  Safe to call from any
        timer or graph-change callback.
        """
        if fqn in self._pending_param_fetches:
            return

        list_client = self.create_client(ListParameters, f"{fqn}/list_parameters")
        if not list_client.service_is_ready():
            self.destroy_client(list_client)
            self._nodes_no_param_service.add(fqn)
            return

        self._pending_param_fetches.add(fqn)
        # Parameter events for this node arriving before the answer land here.
        # Dropping the entry (the node stopped, or fetching was turned off)
        # makes the fetch's answer be ignored.
        changes = self._param_fetch_changes[fqn] = {}
        req = ListParameters.Request()
        req.depth = 10
        future = list_client.call_async(req)

        def _still_current():
            return self._param_fetch_changes.get(fqn) is changes

        def _on_list(fut):
            self.destroy_client(list_client)
            if not _still_current():
                return
            response = fut.result()
            if response is None or not response.result.names:
                self._store_fetched_parameters(fqn, {})  # fetched but empty — stop retrying
                return
            param_names = list(response.result.names)
            get_client = self.create_client(GetParameters, f"{fqn}/get_parameters")
            get_req = GetParameters.Request()
            get_req.names = param_names
            get_future = get_client.call_async(get_req)

            def _on_get(gfut):
                self.destroy_client(get_client)
                if not _still_current():
                    return
                get_resp = gfut.result()
                if get_resp is None:
                    self._pending_param_fetches.discard(fqn)
                    del self._param_fetch_changes[fqn]
                    return
                params = {}
                for name, value in zip(param_names, get_resp.values):
                    try:
                        params[name] = parameter_value_to_python(value)
                    except Exception:
                        pass
                self._store_fetched_parameters(fqn, params)

            get_future.add_done_callback(_on_get)

        future.add_done_callback(_on_list)

    def _store_fetched_parameters(self, fqn: str, params: dict):
        """Cache a finished fetch, with the changes made during it on top."""
        self._pending_param_fetches.discard(fqn)
        changes = self._param_fetch_changes.pop(fqn)
        params = _with_changes(params, changes)
        self._node_parameter_cache[fqn] = params
        self._graph_dirty = True
        self.get_logger().debug(f"[params] cached {len(params)} params for {fqn}")
