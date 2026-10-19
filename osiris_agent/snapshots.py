"""One-shot captures: single-message LiDAR, live topic and bundle snapshots requested by the gateway."""

import asyncio
import base64
import json
import math
import threading
import time

from rosidl_runtime_py import message_to_ordereddict
from rosidl_runtime_py.utilities import get_message

from .payload_encoding import BINARY_MARKER_KEY, BINARY_PAYLOAD_FIELD, BINARY_PAYLOAD_TYPES
from .qos import matching_qos
from .tf_listener import TfListener, resolve_tf_namespace, tf_namespace_published

# A capture's TF listener is new, so the LiDAR's mount transform (latched on
# tf_static) can arrive just after the first scan. It gets at most this long;
# with the 3 s capture timeout that stays within the gateway's 4 s wait.
LIDAR_TF_WAIT_S = 0.5
LIDAR_TF_RETRY_S = 0.05


class SnapshotMixin:

    def _handle_lidar_snapshot_request(self, data: dict):
        """Fire-once LaserScan capture: subscribe, wait for exactly one
        message, unsubscribe, relay it back. Deliberately separate from
        _subscribe_to_topic/_topic_subs (the continuous gateway-requested
        subscription path) — /scan only needs a point-in-time read for wall
        line-fitting, not continuous logging, and keeping it out of
        _topic_subs means it never counts against MAX_SUBSCRIPTIONS or shows
        up in bridge_subscriptions.
        """
        topic_name = data.get('topic') or '/scan'
        request_id = data.get('request_id', '')
        timeout_s = 3.0

        types = dict(self.get_topic_names_and_types()).get(topic_name)
        if not types:
            self.get_logger().warning(f"[lidar_snapshot] topic not found: {topic_name}")
            self._send_lidar_snapshot_failed(request_id, 'topic_not_found')
            return
        if 'sensor_msgs/msg/LaserScan' not in types:
            self.get_logger().warning(f"[lidar_snapshot] {topic_name} is not a LaserScan ({types})")
            self._send_lidar_snapshot_failed(request_id, 'not_a_laserscan')
            return

        msg_class = get_message(types[0])
        lock = threading.Lock()
        state = {'done': False}
        sub_holder = {}

        # Best-effort live TF lookup, started alongside the /scan
        # subscription so /tf_static (latched, arrives near-instantly to a
        # fresh subscriber) has the whole scan-capture window to show up
        # before on_scan actually needs it, plus a moment more if the scan
        # still comes first (_front_angle_when_ready). See
        # _start_lidar_tf_listener for why this is its own Buffer/Listener
        # rather than self._tf_tree.
        tf_buffer, tf_holder = self._start_lidar_tf_listener()

        # Guards against both the scan callback and the timeout firing —
        # whichever gets here first wins, the other is a no-op. Both can run
        # on different threads (rclpy executor thread vs. the timer's own
        # thread), so this needs the lock, not just a plain flag check.
        def finish(success, msg=None, reason=None, front_angle_rad=None):
            with lock:
                # A scan already in hand isn't timed out while TF catches up.
                if state['done'] or (not success and state.get('scan')):
                    return
                state['done'] = True
            sub = sub_holder.pop('sub', None)
            if sub is not None:
                # Same destroy_subscription call _unsubscribe_from_topic already
                # makes in topic_bridge.py — just invoked from inside the
                # subscription's own callback here instead of a separate
                # gateway-driven unsubscribe.
                self.destroy_subscription(sub)
            timer = sub_holder.pop('timer', None)
            if timer is not None:
                timer.cancel()
            self._stop_lidar_tf_listener(tf_holder)
            if success:
                self._send_lidar_snapshot_result(request_id, topic_name, msg, front_angle_rad)
            else:
                self._send_lidar_snapshot_failed(request_id, reason)

        def on_scan(msg):
            with lock:
                if state['done'] or state.get('scan'):
                    return
                state['scan'] = True
            self._front_angle_when_ready(
                tf_buffer, tf_holder, msg.header.frame_id,
                lambda front_angle_rad: finish(True, msg=msg, front_angle_rad=front_angle_rad))

        sub_holder['sub'] = self.create_subscription(msg_class, topic_name, on_scan, matching_qos(self, topic_name, depth=1))

        timer = threading.Timer(timeout_s, lambda: finish(False, reason='timeout'))
        timer.daemon = True
        sub_holder['timer'] = timer
        timer.start()

    @staticmethod
    def _wrap_to_pi(angle: float) -> float:
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle <= -math.pi:
            angle += 2 * math.pi
        return angle

    def _front_angle_when_ready(self, tf_buffer, tf_holder, scan_frame_id: str, deliver):
        """deliver(front_angle_rad) as soon as TF answers, or with None once
        it can't: retried for up to LIDAR_TF_WAIT_S while the robot publishes
        TF but the scan's frame hasn't reached the capture's new buffer yet.
        A robot without TF, or without a path from the scan to base_link,
        gets its answer at once."""
        deadline = time.monotonic() + LIDAR_TF_WAIT_S

        def attempt():
            front_angle_rad = self._lookup_lidar_front_angle(tf_buffer, scan_frame_id)
            if (front_angle_rad is None and time.monotonic() < deadline
                    and self._lidar_frame_pending(tf_buffer, tf_holder, scan_frame_id)):
                retry = threading.Timer(LIDAR_TF_RETRY_S, attempt)
                retry.daemon = True
                retry.start()
                return
            deliver(front_angle_rad)

        attempt()

    def _lidar_frame_pending(self, tf_buffer, tf_holder, scan_frame_id: str) -> bool:
        """Whether TF for the scan's frame can still arrive: the capture's
        listener is running, TF is published where it reads, and the frame
        isn't in its buffer yet."""
        listener = tf_holder.get('listener')
        if tf_buffer is None or listener is None or not scan_frame_id:
            return False
        if not tf_namespace_published(self, listener.namespace):
            return False
        try:
            return scan_frame_id not in tf_buffer._getFrameStrings()
        except Exception:
            return False

    def _lookup_lidar_front_angle(self, tf_buffer, scan_frame_id: str):
        """Best-effort: the robot's true front, expressed as an angle in the
        LiDAR's own raw scan coordinates. Angle 0 in a LaserScan always
        points along the sensor frame's own local +X axis — if that frame
        is itself mounted rotated by some yaw relative to base_link (a real
        situation on custom builds, not just a theoretical one), true front
        ends up at -yaw in the scan's own numbering, not at 0.

        Returns None on any failure (no listener, frame not published yet,
        base_link doesn't exist) — this only ever reports what TF actually
        said, never a fallback guess. The caller/gateway decides what to do
        with "unknown".
        """
        if tf_buffer is None or not scan_frame_id:
            return None
        try:
            from rclpy.time import Time as RclpyTime
            # No timeout arg: this is a non-blocking cache read against
            # whatever /tf_static has already delivered, not a spin-and-wait
            # call — safe to call from inside a message callback.
            t = tf_buffer.lookup_transform('base_link', scan_frame_id, RclpyTime())
        except Exception:
            return None
        q = t.transform.rotation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        return self._wrap_to_pi(-yaw)

    @staticmethod
    def _json_safe_floats(values):
        """Replaces inf/-inf/nan with None. json.dumps happily emits the bare
        Infinity/NaN tokens for these (Python-specific, not valid JSON), which
        makes the gateway's JSON.parse throw on the whole message — and
        LaserScan.ranges is exactly where this bites, since out-of-range
        readings are routinely inf, not just an edge case."""
        return [v if isinstance(v, (int, float)) and math.isfinite(v) else None for v in values]

    def _send_lidar_snapshot_result(self, request_id: str, topic_name: str, msg, front_angle_rad=None):
        if not self.loop:
            return
        data = message_to_ordereddict(msg)
        if 'ranges' in data:
            data['ranges'] = self._json_safe_floats(data['ranges'])
        if 'intensities' in data:
            data['intensities'] = self._json_safe_floats(data['intensities'])
        payload = {
            'type': 'lidar_snapshot_result',
            'request_id': request_id,
            'topic': topic_name,
            'data': data,
            'timestamp': time.time(),
        }
        # Only present when a live TF lookup actually succeeded — the
        # gateway falls back to its own cached/default value otherwise, so
        # this must be omitted rather than sent as 0 when unknown.
        if front_angle_rad is not None:
            payload['front_angle_rad'] = front_angle_rad
        asyncio.run_coroutine_threadsafe(
            self._send_queue.put(json.dumps(payload)),
            self.loop,
        )

    def _send_lidar_snapshot_failed(self, request_id: str, reason: str):
        if not self.loop:
            return
        asyncio.run_coroutine_threadsafe(
            self._send_queue.put(json.dumps({
                'type': 'lidar_snapshot_failed',
                'request_id': request_id,
                'reason': reason,
                'timestamp': time.time(),
            })),
            self.loop,
        )

    @staticmethod
    def _json_safe_deep(value):
        """Recursively replaces inf/-inf/nan with None throughout an
        arbitrary nested dict/list structure, as message_to_ordereddict
        produces for ANY message type — same reasoning as _json_safe_floats
        (LaserScan.ranges), generalized because a generic topic snapshot has
        no fixed field name to special-case; any float field on any message
        type could be non-finite."""
        if isinstance(value, float):
            return value if math.isfinite(value) else None
        if isinstance(value, dict):
            return {k: SnapshotMixin._json_safe_deep(v) for k, v in value.items()}
        if isinstance(value, (list, tuple)):
            return [SnapshotMixin._json_safe_deep(v) for v in value]
        return value

    def _start_one_shot_capture(self, topic_name, on_message, on_timeout, on_not_found, timeout_s=3.0):
        """Shared subscribe-wait-one-message-unsubscribe mechanics, used by
        _handle_topic_live_snapshot_request and by _handle_bundle_capture_
        request, which runs several of these in parallel behind a shared
        completion barrier instead of duplicating the subscribe/timeout/
        cleanup dance per capture. Whichever of on_message/on_timeout fires
        is entirely this function's own business — the caller's callbacks
        just react, they don't need to track the subscription or timer
        themselves. on_not_found fires synchronously, before anything is
        subscribed, if the topic doesn't currently exist."""
        types = dict(self.get_topic_names_and_types()).get(topic_name)
        if not types:
            on_not_found()
            return

        msg_class = get_message(types[0])
        lock = threading.Lock()
        state = {'done': False}
        holder = {}

        # Same "whichever fires first wins" guard as the lidar snapshot's
        # own finish() — the message callback and the timeout can each fire
        # from a different thread.
        def finish(success, msg=None):
            with lock:
                if state['done']:
                    return
                state['done'] = True
            sub = holder.pop('sub', None)
            if sub is not None:
                self.destroy_subscription(sub)
            timer = holder.pop('timer', None)
            if timer is not None:
                timer.cancel()
            if success:
                on_message(msg)
            else:
                on_timeout()

        holder['sub'] = self.create_subscription(msg_class, topic_name, lambda msg: finish(True, msg), matching_qos(self, topic_name, depth=1))

        timer = threading.Timer(timeout_s, lambda: finish(False))
        timer.daemon = True
        holder['timer'] = timer
        timer.start()

    def _start_lidar_tf_listener(self):
        """Best-effort TF listener for a single scan capture, used by
        _handle_lidar_snapshot_request and by _handle_bundle_capture_request's
        lidar_wall_fit capture. Deliberately its own Buffer/
        Listener, not self._tf_tree: that collector only exists while TF tree
        monitoring is enabled, and a scan capture has to work either way.
        Returns (tf_buffer, holder); tf_buffer is None on any failure. The
        caller releases it with _stop_lidar_tf_listener(holder) once done."""
        tf_buffer = None
        tf_holder = {}
        try:
            import tf2_ros
            candidate_buffer = tf2_ros.Buffer()
            tf_holder['listener'] = TfListener(candidate_buffer, self, resolve_tf_namespace(self))
            tf_buffer = candidate_buffer
        except Exception as e:
            tf_buffer = None
            tf_holder.pop('listener', None)
            self.get_logger().debug(f"[tf_listener] unavailable: {e}")
        return tf_buffer, tf_holder

    @staticmethod
    def _stop_lidar_tf_listener(tf_holder):
        """Unsubscribe a capture's TF listener. Popping first makes a second
        call (a timeout racing the scan) a no-op."""
        listener = tf_holder.pop('listener', None)
        if listener is not None:
            listener.unregister()

    def _handle_topic_live_snapshot_request(self, data: dict):
        """Fire-once capture for ANY topic: subscribe, wait for exactly one
        message, unsubscribe, relay it back. Like
        _handle_lidar_snapshot_request, minus the LaserScan type check and TF
        front-angle lookup (those are wall-fit-specific), so the AI can read
        a topic's current value (e.g. /odom) without paying for a lasting
        subscription and continuous logging when it only needs one moment
        in time. Deliberately separate from _subscribe_to_topic/_topic_subs
        (the continuous gateway-requested subscription path) for the same
        reason the lidar one is: never counts against MAX_SUBSCRIPTIONS,
        never shows up in bridge_subscriptions, never persisted as a log
        row — this is a pure ephemeral request/response value, not meant to
        be a durable history."""
        topic_name = data.get('topic')
        request_id = data.get('request_id', '')

        if not topic_name:
            self._send_topic_live_snapshot_failed(request_id, 'topic_required')
            return

        self._start_one_shot_capture(
            topic_name,
            on_message=lambda msg: self._send_topic_live_snapshot_result(request_id, topic_name, msg),
            on_timeout=lambda: self._send_topic_live_snapshot_failed(request_id, 'timeout'),
            on_not_found=lambda: self._send_topic_live_snapshot_failed(request_id, 'topic_not_found'),
        )

    def _handle_bundle_capture_request(self, data: dict):
        """Captures any number of topics + optionally a lidar wall-fit's raw
        scan, all in parallel, joined into ONE combined result once every
        requested capture has resolved (success or failure each). This lets
        the AI correlate values captured together exactly, instead of by
        timestamp proximity, which is ambiguous once more than one run
        happens in the same session.

        Each capture reuses _start_one_shot_capture — this only coordinates
        them behind a shared pending set, it doesn't reimplement the
        subscribe/timeout/cleanup mechanics. The wall-fit computation itself
        does NOT happen here — this only captures the raw /scan message and,
        best-effort, its TF front-angle; the gateway resolves the actual
        heading/distance, same division of responsibility as the dedicated
        lidar-snapshot path.

        A captured message that cannot be encoded fails only its own capture
        (reason 'encode_failed'); the rest of the bundle is still sent. A
        lidar_wall_fit topic that is not a LaserScan fails with
        'not_a_laserscan' without being subscribed to.
        """
        request_id = data.get('request_id', '')
        topics = data.get('topics') or []
        lidar_wall_fit = data.get('lidar_wall_fit')
        lidar_topic = (lidar_wall_fit or {}).get('topic') or '/scan'

        lock = threading.Lock()
        results = {}
        pending = set(topics)
        if lidar_wall_fit is not None:
            pending.add('lidar_wall_fit')

        if not pending:
            self._send_bundle_capture_result(request_id, {})
            return

        def resolve(name, success, value=None, reason=None):
            with lock:
                if name not in pending:
                    return
                pending.discard(name)
                results[name] = {'status': 'success', **(value or {})} if success else {'status': 'failed', 'reason': reason}
                still_pending = bool(pending)
            if not still_pending:
                self._send_bundle_capture_result(request_id, results)

        # The capture handlers run inside a subscription callback, where an
        # exception would escape to the executor and stop the agent.
        def _on_capture_msg(name, msg):
            try:
                encoded = self._encode_bundle_capture_data(name, msg)
            except Exception as e:
                self.get_logger().warning(f"[bundle_capture] {name}: could not encode the captured message: {e}")
                resolve(name, False, reason='encode_failed')
                return
            resolve(name, True, {'data': encoded})

        for t in topics:
            self._start_one_shot_capture(
                t,
                on_message=lambda msg, t=t: _on_capture_msg(t, msg),
                on_timeout=lambda t=t: resolve(t, False, reason='timeout'),
                on_not_found=lambda t=t: resolve(t, False, reason='topic_not_found'),
            )

        # A topic that exists but is not a LaserScan gets the same answer as
        # the dedicated lidar-snapshot path; a missing one is reported by
        # _start_one_shot_capture as topic_not_found, also the same.
        lidar_types = dict(self.get_topic_names_and_types()).get(lidar_topic) if lidar_wall_fit is not None else None
        if lidar_types and 'sensor_msgs/msg/LaserScan' not in lidar_types:
            self.get_logger().warning(f"[bundle_capture] lidar_wall_fit: {lidar_topic} is not a LaserScan ({lidar_types})")
            resolve('lidar_wall_fit', False, reason='not_a_laserscan')
        elif lidar_wall_fit is not None:
            tf_buffer, tf_holder = self._start_lidar_tf_listener()

            def _on_lidar_msg(msg):
                def deliver(front_angle_rad):
                    try:
                        raw_scan = self._json_safe_deep(message_to_ordereddict(msg))
                    except Exception as e:
                        self.get_logger().warning(f"[bundle_capture] lidar_wall_fit: could not encode the scan: {e}")
                        resolve('lidar_wall_fit', False, reason='encode_failed')
                        return
                    finally:
                        self._stop_lidar_tf_listener(tf_holder)
                    resolve('lidar_wall_fit', True, {
                        'raw_scan': raw_scan,
                        'front_angle_rad': front_angle_rad,
                    })

                try:
                    frame_id = msg.header.frame_id
                except Exception as e:
                    self._stop_lidar_tf_listener(tf_holder)
                    self.get_logger().warning(f"[bundle_capture] lidar_wall_fit: could not encode the scan: {e}")
                    resolve('lidar_wall_fit', False, reason='encode_failed')
                    return
                self._front_angle_when_ready(tf_buffer, tf_holder, frame_id, deliver)

            def _on_lidar_timeout():
                self._stop_lidar_tf_listener(tf_holder)
                resolve('lidar_wall_fit', False, reason='timeout')

            def _on_lidar_not_found():
                self._stop_lidar_tf_listener(tf_holder)
                resolve('lidar_wall_fit', False, reason='topic_not_found')

            self._start_one_shot_capture(lidar_topic, _on_lidar_msg, _on_lidar_timeout, _on_lidar_not_found)

    def _encode_bundle_capture_data(self, topic_name: str, msg) -> dict:
        """Converts one bundle-captured message to a JSON-safe dict, with an
        Image/CompressedImage frame embedded as an inline base64 string +
        __osiris_binary__ marker (see _capture_image_as_base64) instead of a
        giant array of decimal ints - the same problem
        _send_topic_live_snapshot_result handles for a single snapshot (see
        its own docstring), reached here by a different path since
        _handle_bundle_capture_request builds each capture's data inline
        rather than going through that function.

        Deliberately NOT a separate binary WS frame the way the continuous
        topic_data stream and the single-topic one-shot snapshot both do it
        - a bundle joins several topics into ONE combined message, and the
        protocol only pairs one pending binary frame per JSON header, so it
        has no room for several at once without a protocol extension.
        Base64 inline costs ~1.37x over raw bytes, same as the gateway's own
        relay - irrelevant for an occasional labeled capture (max 10 topics,
        never a 10Hz stream).

        Image/CompressedImage get the base64 treatment above.
        PointCloud2 gets the same trim/cap _on_topic_msg applies to the
        continuous stream (see the elif below) - still a plain int list
        afterward, just a smaller one; no compression step exists for it
        the way JPEG is for Image. OccupancyGrid is left untouched, as in
        every other capture path.
        """
        data = message_to_ordereddict(msg)
        types = dict(self.get_topic_names_and_types()).get(topic_name)
        msg_type = types[0] if types else None
        encoded = self._capture_image_as_base64(topic_name, data, msg_type)
        if encoded is not None:
            base64_str, marker = encoded
            data[BINARY_PAYLOAD_FIELD] = base64_str
            data[BINARY_MARKER_KEY] = marker
        elif msg_type == 'sensor_msgs/msg/PointCloud2':
            # Same trim/cap _on_topic_msg applies to the continuous stream -
            # a bundled point cloud is just as oversized otherwise, and
            # unlike the other two capture paths this one has nowhere to
            # put a binary frame at all (see this function's own docstring),
            # so an untrimmed cloud here goes straight into the JSON result
            # at full size. Global defaults only, same as _capture_image_as_base64
            # uses for JPEG quality/dimension - no per-call override for a
            # bundle capture.
            raw = data.get(BINARY_PAYLOAD_FIELD)
            if raw is not None:
                payload = bytes(b & 0xFF for b in raw)
                max_points = self._resolve_pointcloud_max_points()
                keep_intensity = self.get_parameter('pointcloud_keep_intensity').get_parameter_value().bool_value
                keep_rgb = self.get_parameter('pointcloud_keep_rgb').get_parameter_value().bool_value
                downsampled = self._downsample_pointcloud(payload, data, max_points, keep_intensity, keep_rgb, topic_name)
                if downsampled is not None:
                    payload, new_fields, new_point_step, out_points = downsampled
                    data['fields'] = new_fields
                    data['point_step'] = new_point_step
                    data['width'] = out_points
                    data['height'] = 1
                    data['row_step'] = new_point_step * out_points
                    # No binary frame/marker for this type (see _on_topic_msg's
                    # own comment) - stays a plain list of decimal ints, just
                    # a smaller one now.
                    data[BINARY_PAYLOAD_FIELD] = list(payload)
        return self._json_safe_deep(data)

    def _capture_image_as_base64(self, topic_name: str, data: dict, msg_type: str):
        """For an Image/CompressedImage message already run through
        message_to_ordereddict, returns (base64_str, marker) - marker
        shaped exactly like what a client already gets from the gateway's
        own binary reconstruction (format/width/height/len/encoding), so
        any existing consumer's `__osiris_binary__.encoding === 'base64'`
        check works identically regardless of which path produced it. Returns None for any other type, or if an Image's
        JPEG re-encode itself fails (unsupported encoding, corrupt frame,
        Pillow unavailable) - callers leave `data` untouched in that case,
        same fallback-to-raw behavior _on_topic_msg/_send_topic_live_snapshot_
        result already have.

        Uses the agent-wide default quality/dimension, same as
        _send_topic_live_snapshot_result - a one-shot/bundle capture isn't
        tied to a specific subscription's topic_limits override.
        """
        # Checked before touching `data`: other message types can have a
        # `data` field that is not a byte array (std_msgs/String, Float32, ...).
        if msg_type not in ('sensor_msgs/msg/Image', 'sensor_msgs/msg/CompressedImage'):
            return None
        raw = data.get(BINARY_PAYLOAD_FIELD)
        if raw is None:
            return None
        payload_bytes = bytes(b & 0xFF for b in raw)
        if msg_type == 'sensor_msgs/msg/Image':
            quality = self.get_parameter('image_jpeg_quality').get_parameter_value().integer_value
            max_dimension = self.get_parameter('image_max_dimension').get_parameter_value().integer_value
            reencoded = self._reencode_image_jpeg(
                payload_bytes, data.get('width'), data.get('height'), data.get('encoding'),
                quality, max_dimension, topic_name,
            )
            if reencoded is None:
                return None
            payload_bytes, out_width, out_height = reencoded
            marker = {'format': 'jpeg', 'width': out_width, 'height': out_height}
        else:
            marker = {'format': data.get('format') or 'unknown'}
        marker['len'] = len(payload_bytes)
        marker['encoding'] = 'base64'
        return base64.b64encode(payload_bytes).decode('ascii'), marker

    def _send_bundle_capture_result(self, request_id: str, captures: dict):
        if not self.loop:
            return
        payload = {
            'type': 'bundle_capture_result',
            'request_id': request_id,
            'captures': captures,
            'timestamp': time.time(),
        }
        asyncio.run_coroutine_threadsafe(
            self._send_queue.put(json.dumps(payload)),
            self.loop,
        )

    def _send_bundle_capture_failed(self, request_id: str, reason: str):
        if not self.loop:
            return
        asyncio.run_coroutine_threadsafe(
            self._send_queue.put(json.dumps({
                'type': 'bundle_capture_failed',
                'request_id': request_id,
                'reason': reason,
                'timestamp': time.time(),
            })),
            self.loop,
        )

    def _send_topic_live_snapshot_result(self, request_id: str, topic_name: str, msg):
        """Sends a one-shot capture's result. Binary-payload types (Image/
        CompressedImage/PointCloud2/OccupancyGrid) get the same treatment
        _on_topic_msg gives the continuous topic_data stream - a raw pixel/
        point byte array as a JSON array of decimal ints is exactly the
        3-6x-inflated waste here it would be there, and for Image
        specifically a giant int array isn't something a vision model can
        even look at; it needs to actually be a JPEG. Deliberately no budget
        check here - a
        one-shot capture isn't part of the rate/byte-budget system that
        governs the continuous stream, and shouldn't be silently dropped by
        a cap that has nothing to do with this specific request. Type is
        re-resolved fresh (this topic was never added to _topic_msg_types -
        that's populated by the continuous, gateway-requested subscription
        path only) via the same lookup _start_one_shot_capture itself uses.
        """
        if not self.loop:
            return
        data = message_to_ordereddict(msg)

        types = dict(self.get_topic_names_and_types()).get(topic_name)
        msg_type = types[0] if types else None
        signed = BINARY_PAYLOAD_TYPES.get(msg_type)
        raw = data.get(BINARY_PAYLOAD_FIELD) if signed is not None else None
        if raw is not None:
            payload_bytes = bytes(b & 0xFF for b in raw)
            image_marker_extra = None
            if msg_type == 'sensor_msgs/msg/Image':
                quality = self.get_parameter('image_jpeg_quality').get_parameter_value().integer_value
                max_dimension = self.get_parameter('image_max_dimension').get_parameter_value().integer_value
                reencoded = self._reencode_image_jpeg(
                    payload_bytes, data.get('width'), data.get('height'), data.get('encoding'),
                    quality, max_dimension, topic_name,
                )
                if reencoded is not None:
                    payload_bytes, out_width, out_height = reencoded
                    image_marker_extra = {'format': 'jpeg', 'width': out_width, 'height': out_height}
            elif msg_type == 'sensor_msgs/msg/CompressedImage':
                image_marker_extra = {'format': data.get('format') or 'unknown'}
            elif msg_type == 'sensor_msgs/msg/PointCloud2':
                # Same trim/cap _on_topic_msg applies to the continuous
                # stream - a one-shot capture of a real point cloud is just
                # as oversized otherwise. Global defaults only, same as the
                # JPEG quality/dimension above - a one-shot capture doesn't
                # go through _topic_limit_overrides.
                max_points = self._resolve_pointcloud_max_points()
                keep_intensity = self.get_parameter('pointcloud_keep_intensity').get_parameter_value().bool_value
                keep_rgb = self.get_parameter('pointcloud_keep_rgb').get_parameter_value().bool_value
                downsampled = self._downsample_pointcloud(payload_bytes, data, max_points, keep_intensity, keep_rgb, topic_name)
                if downsampled is not None:
                    payload_bytes, new_fields, new_point_step, out_points = downsampled
                    data['fields'] = new_fields
                    data['point_step'] = new_point_step
                    data['width'] = out_points
                    data['height'] = 1
                    data['row_step'] = new_point_step * out_points

            marker = {'len': len(payload_bytes), 'signed': signed}
            if image_marker_extra is not None:
                marker.update(image_marker_extra)
            # Replace the (potentially huge) raw array with the small marker
            # dict BEFORE the recursive NaN-scrub below, not after - no
            # reason to walk hundreds of thousands of already-extracted ints
            # a second time.
            data[BINARY_PAYLOAD_FIELD] = {BINARY_MARKER_KEY: marker}
            data = self._json_safe_deep(data)
            header = {
                'type': 'topic_live_snapshot_result',
                'request_id': request_id,
                'topic': topic_name,
                'data': data,
                'timestamp': time.time(),
            }
            self._enqueue_binary_topic_data(header, payload_bytes)
            return

        data = self._json_safe_deep(data)
        payload = {
            'type': 'topic_live_snapshot_result',
            'request_id': request_id,
            'topic': topic_name,
            'data': data,
            'timestamp': time.time(),
        }
        asyncio.run_coroutine_threadsafe(
            self._send_queue.put(json.dumps(payload)),
            self.loop,
        )

    def _send_topic_live_snapshot_failed(self, request_id: str, reason: str):
        if not self.loop:
            return
        asyncio.run_coroutine_threadsafe(
            self._send_queue.put(json.dumps({
                'type': 'topic_live_snapshot_failed',
                'request_id': request_id,
                'reason': reason,
                'timestamp': time.time(),
            })),
            self.loop,
        )
