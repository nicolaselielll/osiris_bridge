"""Gateway connection: WebSocket connect, auth and reconnect, the send queue, incoming message dispatch and initial_state."""

import asyncio
import collections
import json
import random
import threading
import time

import websockets
from websockets.exceptions import ConnectionClosed

from osiris_agent import __version__ as AGENT_VERSION

RECONNECT_INITIAL_DELAY    = 1     # seconds
RECONNECT_MAX_DELAY        = 30    # seconds

# Goal events from while the agent isn't connected (its first moments after a
# restart, when the action servers' kept statuses arrive, or a dropped
# connection) are kept and sent once it is, after the initial state. Without
# them a goal that ends meanwhile, or one a restarted agent finds finished,
# never gets its end. Until the kept ones are queued, new goal events are kept
# behind them too, so a goal's statuses always reach the gateway in order. At
# most this many; the oldest are dropped first.
PENDING_GOAL_EVENTS_MAX = 500
_pending_lock = threading.Lock()

# What a user can do when the gateway turns the agent away, by close code.
GATEWAY_REFUSAL_HINTS = {
    4429: "the workspace's storage is full. Delete old data in the web app (robot menu > Delete data) to reconnect",
    4500: 'the gateway could not check the storage limit, retrying',
    1008: 'the agent token is missing, invalid or was rotated. Rotate it in the web app (robot menu > Agent) and set OSIRIS_AUTH_TOKEN to the new token',
}


class ConnectionMixin:

    def _run_ws_client(self):
        self.loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self.loop)
        self._send_queue = asyncio.Queue()
        self.loop.run_until_complete(self._client_loop_with_reconnect())

    async def _client_loop_with_reconnect(self):
        delay = RECONNECT_INITIAL_DELAY
        while self.context.ok():
            self._connected_since = None
            try:
                await self._client_loop()
            except Exception as e:
                if self.context.ok():
                    self.get_logger().warning(f"WebSocket error: {e}")
            # A connection that stayed up longer than the longest wait was a healthy
            # one, so the next attempt starts over. Failed attempts and short
            # connections keep backing off, so a gateway that drops the agent right
            # after accepting it is not hit every second.
            if self._connected_since is not None and time.monotonic() - self._connected_since >= RECONNECT_MAX_DELAY:
                delay = RECONNECT_INITIAL_DELAY
            # Jitter spreads out agents that dropped together, such as on a gateway restart.
            wait = delay * random.uniform(0.75, 1.25)
            if self.context.ok():
                self.get_logger().info(f'Reconnecting in {wait:.1f}s')
            await asyncio.sleep(wait)
            delay = min(delay * 2, RECONNECT_MAX_DELAY)

    async def _client_loop(self):
        send_task = None
        await asyncio.to_thread(self._wait_for_graph_to_settle)
        self.get_logger().info('Connecting to gateway...')
        try:
            async with websockets.connect(self.ws_url) as ws:
                try:
                    auth_msg = await ws.recv()
                    auth_data = json.loads(auth_msg)
                except ConnectionClosed as closed:
                    self._log_gateway_refusal(closed)
                    return
                except Exception:
                    self.get_logger().error('Failed to receive auth response from gateway')
                    return

                if not auth_data or auth_data.get('type') != 'auth_success':
                    error_msg = auth_data.get('message', 'unknown') if auth_data else 'no response'
                    self.get_logger().error(f'Authentication failed: {error_msg}')
                    return

                self.get_logger().info('Connected and authenticated to gateway')
                self._connected_since = time.monotonic()
                self.ws = ws
                send_task = asyncio.create_task(self._send_loop(ws))

                await self._send_initial_state()
                await self._send_pending_goal_events()
                await self._receive_loop(ws)
        finally:
            # Goal events are kept again until the next connection's are sent.
            self._stop_goal_events_live()
            # Awaited whether it is still running or already finished, so a
            # result it ended with is always collected.
            if send_task:
                if not send_task.done():
                    send_task.cancel()
                try:
                    await send_task
                except (asyncio.CancelledError, Exception):
                    pass
            if self.ws is not None:
                if self.ws.close_code == 4429:
                    self.get_logger().warning(f'Storage limit reached: {GATEWAY_REFUSAL_HINTS[4429]}')
                else:
                    self.get_logger().warning('Disconnected from gateway')
            self.ws = None

    def _log_gateway_refusal(self, closed):
        """Log why the gateway closed the connection before authenticating it,
        with what to do about it when the close code says."""
        frame = getattr(closed, 'rcvd', None)
        code = getattr(frame, 'code', None)
        reason = getattr(frame, 'reason', '') or 'no reason given'
        hint = GATEWAY_REFUSAL_HINTS.get(code)
        self.get_logger().error(
            f'Gateway refused the connection: {reason} (code {code})' + (f': {hint}' if hint else ''))

    async def _send_loop(self, ws):
        while True:
            item = await self._send_queue.get()
            # A tuple is a topic_data header and its binary frame (see _put_pair).
            frames = item if isinstance(item, tuple) else (item,)
            try:
                for frame in frames:
                    await ws.send(frame)
            except Exception as e:
                # A failed send ends the connection, so the receive loop ends too
                # and the agent reconnects, rather than staying connected without
                # sending anything.
                self.get_logger().error(f"WS send failed: {e}")
                try:
                    await ws.close()
                except Exception:
                    pass
                return

    async def _receive_loop(self, ws):
        async for raw in ws:
            if not self.context.ok():
                break
            try:
                data = json.loads(raw)
            except json.JSONDecodeError:
                continue
            msg_type = data.get('type')
            if msg_type == 'agent_config':
                # A push with a malformed field is rejected as a whole inside
                # _apply_agent_config, before anything changes. This catch is
                # for anything unexpected while applying a valid one: left
                # uncaught, it would end this loop and be treated as a
                # WebSocket error, tearing down the whole connection.
                try:
                    self._apply_agent_config(data.get('config') or {})
                except Exception as e:
                    self.get_logger().error(f'Failed to apply agent_config: {e}')
            elif msg_type == 'subscribe':
                topic = data.get('topic')
                if topic:
                    self._subscribe_to_topic(topic)
            elif msg_type == 'unsubscribe':
                topic = data.get('topic')
                if topic:
                    self._unsubscribe_from_topic(topic)
            elif msg_type == 'lidar_snapshot_request':
                self._handle_lidar_snapshot_request(data)
            elif msg_type == 'topic_live_snapshot_request':
                self._handle_topic_live_snapshot_request(data)
            elif msg_type == 'bundle_capture_request':
                self._handle_bundle_capture_request(data)
            elif msg_type == 'send_command_request':
                self._handle_send_command_request(data)
            elif msg_type == 'cancel_command_request':
                self._handle_cancel_command_request(data)
            elif msg_type == 'error':
                self.get_logger().warning(f"Gateway error: {data.get('message', '')}")
            elif msg_type == 'bag_start_record':
                topics = data.get('topics', [])
                self.get_logger().info(
                    f"[bag] bag_start_record received  topics={topics or 'all'}"
                )
                self._start_bag_recording(data)
            elif msg_type == 'bag_stop_record':
                self.get_logger().info("[bag] bag_stop_record received")
                await asyncio.to_thread(self._stop_bag_recording)
                # A page that opens later lists the robot's bags from what the
                # gateway kept of the last bag_files, which is otherwise only
                # sent on connect: send it again, with the bag just finished.
                await self._send_bag_files()
            elif msg_type == 'bag_download_request':
                path       = data.get('path', '')
                request_id = data.get('request_id', '')
                upload_url = data.get('upload_url', '')
                self.get_logger().info(
                    f"[bag] bag_download_request received  path={path}  request_id={request_id}"
                )
                asyncio.ensure_future(
                    asyncio.to_thread(self._send_bag_download, path, request_id, upload_url)
                )

    async def _send_initial_state(self):
        # Wait for the first _check_graph_changes tick to populate all caches.
        await asyncio.to_thread(self._initial_scan_complete.wait, 15.0)

        # Reset delta caches so _flush_graph_snapshots treats everything as
        # "unsent" after this reconnect.
        self._last_sent_nodes    = None
        self._last_sent_topics   = None
        self._last_sent_actions  = None
        self._last_sent_services = None
        self._graph_dirty        = True

        nodes, topics, actions, services = await asyncio.to_thread(self._get_graph_snapshot_locked)
        controllers = self._ros2_control.get_controllers_snapshot() if self._ros2_control is not None else []
        hardware    = self._ros2_control.get_hardware_snapshot() if self._ros2_control is not None else []
        telemetry   = self._get_telemetry_snapshot()
        tf_tree     = self._tf_tree.get_snapshot() if self._tf_tree is not None else None
        bt_state    = self._build_startup_bt_state_event()
        bt          = self._bt_snapshot_from_state_event(bt_state)
        initial_timestamp = time.time()

        self._last_sent_nodes    = nodes.copy()
        self._last_sent_topics   = topics.copy()
        self._last_sent_actions  = actions.copy()
        self._last_sent_services = services.copy()

        await self._send_queue.put(json.dumps({
            'type': 'agent_version',
            'version': AGENT_VERSION,
        }))

        await self._send_queue.put(json.dumps({
            'type': 'initial_state',
            'timestamp': initial_timestamp,
            'data': {
                'timestamp': initial_timestamp,
                'graph': {
                    'nodes':       nodes,
                    'topics':      topics,
                    'actions':     actions,
                    'services':    services,
                    'controllers': controllers,
                    'hardware':    hardware,
                },
                'meta':      self._get_initial_state_meta(telemetry),
                'telemetry': telemetry,
                'tf_tree':   tf_tree,
                'bt':        bt,
            },
        }))

        await self._send_queue.put(json.dumps(bt_state))

        if self._cached_bt_tree_event:
            await self._send_queue.put(json.dumps(self._cached_bt_tree_event))
            self._cached_bt_tree_event = None

        await self._send_bridge_subscriptions()
        await self._send_bag_files()

        self.get_logger().info(
            f"Sent initial_state: {len(nodes)} nodes, {len(topics)} topics, "
            f"{len(actions)} actions, {len(services)} services"
        )

    def _enqueue(self, payload: dict):
        """Thread-safe enqueue to the asyncio send queue. A goal event is kept
        until the connection's kept goal events are queued (see
        PENDING_GOAL_EVENTS_MAX); anything else is dropped while not
        connected, as before."""
        if payload.get('type') == 'goal_event':
            with _pending_lock:
                if not getattr(self, '_goal_events_live', False):
                    self._kept_goal_events().append(payload)
                    return
                self._queue_threadsafe(payload)
            return
        if self.ws and self.loop:
            self._queue_threadsafe(payload)

    def _queue_threadsafe(self, payload: dict):
        asyncio.run_coroutine_threadsafe(
            self._send_queue.put(json.dumps(payload)),
            self.loop,
        )

    def _kept_goal_events(self):
        # Callers hold _pending_lock.
        kept = getattr(self, '_pending_goal_events_kept', None)
        if kept is None:
            kept = self._pending_goal_events_kept = collections.deque(maxlen=PENDING_GOAL_EVENTS_MAX)
        return kept

    async def _send_pending_goal_events(self):
        """Queues the goal events kept while not connected, oldest first, and
        from then on lets new ones straight through, all under the lock so none
        slips in between."""
        with _pending_lock:
            kept = self._kept_goal_events()
            while kept:
                self._send_queue.put_nowait(json.dumps(kept.popleft()))
            self._goal_events_live = True

    def _stop_goal_events_live(self):
        with _pending_lock:
            self._goal_events_live = False

    async def _put_pair(self, header: str, payload: bytes):
        # The gateway pairs a binary frame with the header received right
        # before it on the same connection. One queue item, sent back to back
        # by _send_loop, keeps the two together: no other message can land
        # between them, and a connection that closes before or between them
        # consumes the whole item instead of leaving the binary frame to open
        # the next connection.
        await self._send_queue.put((header, payload))

    def _enqueue_binary_topic_data(self, header: dict, payload: bytes):
        """Like _enqueue, but for a topic_data message whose byte-array field
        was pulled out into a raw binary WS frame (see _on_topic_msg) — sends
        the JSON header and that frame as one atomic pair. See _put_pair."""
        if self.ws and self.loop:
            asyncio.run_coroutine_threadsafe(
                self._put_pair(json.dumps(header), payload),
                self.loop,
            )
