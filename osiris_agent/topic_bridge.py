"""Topic bridging: gateway-requested topic subscriptions forwarded as topic_data, within the rate and byte caps."""

import asyncio
import copy
import json
import time
from collections import deque

from rosidl_runtime_py import message_to_ordereddict
from rosidl_runtime_py.utilities import get_message

from .payload_encoding import BINARY_MARKER_KEY, BINARY_PAYLOAD_FIELD, BINARY_PAYLOAD_TYPES
from .qos import matching_qos, qos_key

MAX_SUBSCRIPTIONS          = 100   # hard cap on gateway-requested topic subs


class TopicBridgeMixin:

    def _subscribe_to_topic(self, topic_name: str):
        if not topic_name or not isinstance(topic_name, str):
            return
        with self._topic_subs_lock:
            if topic_name in self._topic_subs:
                return
            if len(self._topic_subs) >= MAX_SUBSCRIPTIONS:
                self.get_logger().error(
                    f"Subscription limit ({MAX_SUBSCRIPTIONS}) reached; "
                    f"cannot subscribe to {topic_name}"
                )
                if self.loop:
                    asyncio.run_coroutine_threadsafe(
                        self._send_subscribe_failed(topic_name, 'subscription_limit_reached'), self.loop
                    )
                return

        types = dict(self.get_topic_names_and_types()).get(topic_name)
        if not types:
            self.get_logger().warning(f"Topic not found: {topic_name}")
            if self.loop:
                asyncio.run_coroutine_threadsafe(
                    self._send_subscribe_failed(topic_name, 'topic_not_found'), self.loop
                )
            return

        msg_class = get_message(types[0])
        qos = matching_qos(self, topic_name, depth=10)
        sub = self.create_subscription(
            msg_class,
            topic_name,
            lambda msg, t=topic_name: self._on_topic_msg(msg, t),
            qos,
        )
        with self._topic_subs_lock:
            self._topic_subs[topic_name] = sub
            self._topic_msg_types[topic_name] = types[0]
            self._topic_sub_qos[topic_name] = qos_key(qos)

        self.get_logger().info(f"Subscribed to {topic_name}")
        # Logged once here, when the setting actually takes effect - not
        # repeated every time a message gets throttled, since a capped
        # topic being throttled is the normal, continuous state for as
        # long as it stays subscribed, not a per-drop event worth logging.
        with self._topic_limit_lock:
            rate_override = self._topic_limit_overrides.get(topic_name, {})
        rate_hz, rate_hz_source = self._resolve_topic_rate_hz(rate_override, types[0])
        if rate_hz > 0:
            self.get_logger().info(f'[topic_data] {topic_name}: capped at {rate_hz:.2f} Hz ({rate_hz_source})')
        if self.loop:
            asyncio.run_coroutine_threadsafe(
                self._send_bridge_subscriptions(), self.loop
            )

    def _refresh_bridge_qos(self, topics):
        """Re-creates a bridged subscription whose publishers changed in a way
        its QoS can't receive from, e.g. a BEST_EFFORT publisher appearing after
        the subscription was made against a RELIABLE one (or none at all)."""
        for topic_name in topics:
            with self._topic_subs_lock:
                sub = self._topic_subs.get(topic_name)
                msg_type = self._topic_msg_types.get(topic_name)
                current = self._topic_sub_qos.get(topic_name)
            if sub is None or not msg_type or not self.get_publishers_info_by_topic(topic_name):
                continue
            qos = matching_qos(self, topic_name, depth=10)
            if qos_key(qos) == current:
                continue
            new_sub = self.create_subscription(
                get_message(msg_type), topic_name,
                lambda msg, t=topic_name: self._on_topic_msg(msg, t), qos,
            )
            with self._topic_subs_lock:
                if self._topic_subs.get(topic_name) is not sub:
                    self.destroy_subscription(new_sub)
                    continue
                self._topic_subs[topic_name] = new_sub
                self._topic_sub_qos[topic_name] = qos_key(qos)
            self.destroy_subscription(sub)
            self.get_logger().info(f'[topic_data] {topic_name}: resubscribed to match its publishers\' QoS')

    def _unsubscribe_from_topic(self, topic_name: str):
        with self._topic_subs_lock:
            sub = self._topic_subs.pop(topic_name, None)
            self._topic_msg_types.pop(topic_name, None)
            self._topic_sub_qos.pop(topic_name, None)
        if sub:
            self.destroy_subscription(sub)
            self._topic_data_throttle.pop(topic_name, None)
            self.get_logger().info(f"Unsubscribed from {topic_name}")
            if self.loop:
                asyncio.run_coroutine_threadsafe(
                    self._send_bridge_subscriptions(), self.loop
                )

    async def _send_bridge_subscriptions(self):
        with self._topic_subs_lock:
            subs = list(self._topic_subs.keys())
        await self._send_queue.put(json.dumps({
            'type': 'bridge_subscriptions',
            'subscriptions': subs,
            'timestamp': time.time(),
        }))

    async def _send_subscribe_failed(self, topic_name: str, reason: str):
        await self._send_queue.put(json.dumps({
            'type': 'subscribe_failed',
            'topic': topic_name,
            'reason': reason,
            'timestamp': time.time(),
        }))

    def _on_topic_msg(self, msg, topic_name: str):
        if not self.ws or not self.loop:
            return

        ts = time.time()
        with self._topic_rate_lock:
            self._topic_rate_timestamps.setdefault(topic_name, deque()).append(ts)

        # Per-topic overrides (set live via agent_config's 'topic_limits',
        # see _apply_agent_config) take precedence over the two defaults
        # below when present for this topic.
        with self._topic_limit_lock:
            override = self._topic_limit_overrides.get(topic_name, {})

        # msg_type/signed only need topic_name and the pre-populated
        # _topic_msg_types map (set at subscription time), so this is
        # resolved up front - both the rate throttle below and the
        # message-conversion skip further down need it.
        with self._topic_subs_lock:
            msg_type = self._topic_msg_types.get(topic_name)
        signed = BINARY_PAYLOAD_TYPES.get(msg_type)

        # Cap how often a given topic's data actually gets forwarded — a fast
        # topic (odom, joint_states, ...) publishing at 30-50+ Hz is far past
        # what's perceptible in Watch/Plot, and every message costs a DB row
        # on the gateway besides. rate_hz itself stays accurate since it's
        # computed from _topic_rate_timestamps above, sampled every receipt
        # regardless of this throttle. Full-fidelity capture, when actually
        # needed, goes through bag recording instead of this live stream.
        # Read fresh (not cached) same as graph_debounce_interval/bag_output_dir
        # — cheap local lookup, and lets a config change take effect without
        # a reconnect. See _subscribe_to_topic for the one-time log of what
        # this resolves to — dropped messages themselves aren't logged here,
        # since being throttled is the normal, continuous, expected state
        # for a capped topic, not a notable event worth a warning every time.
        rate_hz, _ = self._resolve_topic_rate_hz(override, msg_type)
        if rate_hz > 0 and ts - self._topic_data_throttle.get(topic_name, 0.0) < 1.0 / rate_hz:
            return
        self._topic_data_throttle[topic_name] = ts

        # For the byte-array-dominated types (see BINARY_PAYLOAD_TYPES), pull
        # the array out and send it as a raw binary WS frame instead of
        # leaving it for json.dumps to inflate into a comma-separated array
        # of ints. See BINARY_MARKER_KEY for how the gateway pairs the binary
        # frame with this message.
        #
        # The array is read directly off the live message object before
        # message_to_ordereddict runs, and message_to_ordereddict itself
        # runs on a shallow copy with that one field sliced empty
        # (raw[0:0] - same array/list type as raw, so rosidl's property
        # setter accepts it without a type mismatch) rather than on msg
        # directly. Every other field (width, height, fields, point_step,
        # ...) still converts normally from the copy. copy.copy(), not
        # mutating msg directly, since rclpy owns that object beyond this
        # callback. data[BINARY_PAYLOAD_FIELD] gets unconditionally
        # overwritten with the marker dict below regardless of what's in it,
        # so the sliced-empty copy is never sent anywhere.
        raw = None
        msg_for_dict = msg
        if signed is not None:
            raw = getattr(msg, BINARY_PAYLOAD_FIELD, None)
            if raw is not None:
                msg_for_dict = copy.copy(msg)
                setattr(msg_for_dict, BINARY_PAYLOAD_FIELD, raw[0:0])

        data = message_to_ordereddict(msg_for_dict)

        if raw is not None:
            # raw is msg.data itself (array.array/numpy/list depending on
            # rclpy's config for this field), not message_to_ordereddict's
            # output - it yields correctly-signed Python ints on iteration
            # either way (array.array('b',...) for OccupancyGrid's int8[] is
            # signed at the container level). `& 0xFF` takes each one to its
            # two's-complement byte value regardless of signedness — the same
            # bit pattern either way — which is exactly what bytes() needs and
            # exactly what the gateway's Int8Array/Uint8Array reinterprets on
            # the way back using the 'signed' flag below.
            payload = bytes(b & 0xFF for b in raw)

            # Image: re-encode to JPEG before the budget check below, so the
            # cap is enforced against the actual bytes going out, not the raw
            # pixel size this replaces. Falls through with the original raw
            # payload untouched if Pillow isn't installed, the encoding isn't
            # one this recognizes, or the buffer doesn't match the declared
            # dimensions. CompressedImage: already compressed by the camera
            # driver, nothing to re-encode, just marked the same way (see
            # the elif below) for the gateway's benefit. PointCloud2: trimmed
            # to xyz(+intensity/rgb)/point-capped, see _downsample_pointcloud.
            image_marker_extra = None
            if msg_type == 'sensor_msgs/msg/Image':
                quality = override.get('jpeg_quality')
                if quality is None:
                    quality = self.get_parameter('image_jpeg_quality').get_parameter_value().integer_value
                max_dimension = override.get('max_dimension')
                if max_dimension is None:
                    max_dimension = self.get_parameter('image_max_dimension').get_parameter_value().integer_value
                reencoded = self._reencode_image_jpeg(
                    payload, data.get('width'), data.get('height'), data.get('encoding'),
                    quality, max_dimension, topic_name,
                )
                if reencoded is not None:
                    payload, out_width, out_height = reencoded
                    image_marker_extra = {'format': 'jpeg', 'width': out_width, 'height': out_height}
            elif msg_type == 'sensor_msgs/msg/CompressedImage':
                # Already compressed by the driver - nothing to re-encode,
                # just marked the same way a re-encoded Image is so the
                # gateway relays it as base64 instead of reconstructing a
                # JSON array of decimal ints - the same win Image gets, for
                # the same reason
                # (it's already-compressed opaque bytes, not raw pixels a
                # future consumer would index into numerically). No
                # width/height here - CompressedImage's own ROS message
                # doesn't carry them, they're implicit in the compressed
                # bytes themselves.
                image_marker_extra = {'format': data.get('format') or 'unknown'}
            elif msg_type == 'sensor_msgs/msg/PointCloud2':
                # Trim to xyz (+ intensity/rgb if configured) and cap the
                # point count before the budget check below, same reasoning
                # as Image's re-encode above - the cap is enforced against
                # what's actually going out, not the untrimmed size this
                # replaces. No marker changes here (no image_marker_extra) -
                # the result is still a plain binary blob, just smaller,
                # and the gateway reconstructs it as a JSON array of numbers.
                max_points = self._resolve_pointcloud_max_points(override)
                keep_intensity = override.get('pointcloud_keep_intensity')
                if keep_intensity is None:
                    keep_intensity = self.get_parameter('pointcloud_keep_intensity').get_parameter_value().bool_value
                keep_rgb = override.get('pointcloud_keep_rgb')
                if keep_rgb is None:
                    keep_rgb = self.get_parameter('pointcloud_keep_rgb').get_parameter_value().bool_value
                downsampled = self._downsample_pointcloud(payload, data, max_points, keep_intensity, keep_rgb, topic_name)
                if downsampled is not None:
                    payload, new_fields, new_point_step, out_points = downsampled
                    # Metadata has to match the trimmed payload exactly -
                    # fields/point_step/width describe the byte layout a
                    # consumer would use to index into `data`, and row_step
                    # is just point_step*width for an unorganized cloud
                    # (height=1, which stride-subsampling already implies:
                    # whatever 2D structure the original had is gone once
                    # points are dropped non-contiguously).
                    data['fields'] = new_fields
                    data['point_step'] = new_point_step
                    data['width'] = out_points
                    data['height'] = 1
                    data['row_step'] = new_point_step * out_points

            if self._topic_data_over_budget(topic_name, len(payload), ts, override):
                return
            marker = {'len': len(payload), 'signed': signed}
            if image_marker_extra is not None:
                marker.update(image_marker_extra)
            data[BINARY_PAYLOAD_FIELD] = {BINARY_MARKER_KEY: marker}
            self._enqueue_binary_topic_data({
                'type': 'topic_data',
                'topic': topic_name,
                'data': data,
                'timestamp': ts,
            }, payload)
            return

        body = json.dumps({
            'type': 'topic_data',
            'topic': topic_name,
            'data': data,
            'timestamp': ts,
        })
        if self._topic_data_over_budget(topic_name, len(body), ts, override):
            return
        asyncio.run_coroutine_threadsafe(self._send_queue.put(body), self.loop)

    def _topic_data_over_budget(self, topic_name: str, size: int, ts: float, override: dict) -> bool:
        """True if forwarding `size` more bytes for `topic_name` right now
        would exceed that topic's own bytes/sec budget or the connection
        -wide budget shared across every subscribed topic (see the three
        topic_data_max_bytes_per_sec / global_topic_data_max_* parameters
        and self._topic_limit_overrides). Only topic_data competes for
        either budget — graph state, lifecycle events, etc. go through plain
        _enqueue and are never throttled here, on purpose: the point is
        protecting the heavy sensor stream without ever starving the small
        control-plane traffic that the rest of the UI depends on.

        Both budgets are simple 1-second buckets that reset once a full
        second has elapsed since the window opened, rather than a true
        sliding window — coarser, but consistent with the Hz throttle's own
        granularity above and much cheaper than tracking individual message
        timestamps per topic.
        """
        # Recorded unconditionally, before any cap is even read - same
        # "measure the real thing, not what Osiris decided to let through"
        # reasoning as _topic_rate_timestamps in _on_topic_msg. A topic
        # capped hard enough to drop every message would otherwise report
        # 0 B/s, hiding exactly the number someone raising that cap needs.
        with self._topic_rate_lock:
            self._topic_byte_timestamps.setdefault(topic_name, deque()).append((ts, size))

        topic_cap = override.get('max_bytes_per_sec')
        if topic_cap is None:
            topic_cap = self.get_parameter('topic_data_max_bytes_per_sec').get_parameter_value().double_value
        global_byte_cap = self.get_parameter('global_topic_data_max_bytes_per_sec').get_parameter_value().double_value
        global_msg_cap  = self.get_parameter('global_topic_data_max_msgs_per_sec').get_parameter_value().double_value

        t_start, t_bytes = self._topic_byte_bucket.get(topic_name, (ts, 0))
        if ts - t_start >= 1.0:
            t_start, t_bytes = ts, 0

        g_start, g_bytes, g_msgs = self._global_topic_data_bucket
        if ts - g_start >= 1.0:
            g_start, g_bytes, g_msgs = ts, 0, 0

        # Which specific cap tripped, if any - not just a bool, so the drop
        # warning below can name it. Without that, an unexpectedly-low
        # forwarded rate (a topic capped far below its real publish rate,
        # with nothing in the log explaining why) is hard to tell apart from
        # a genuinely slow source, a Hz-throttle, or a processing bottleneck
        # elsewhere.
        reason = None
        if topic_cap > 0 and t_bytes + size > topic_cap:
            reason = f'topic byte budget ({t_bytes + size:.0f}/{topic_cap:.0f} B/s)'
        elif global_byte_cap > 0 and g_bytes + size > global_byte_cap:
            reason = f'global byte budget ({g_bytes + size:.0f}/{global_byte_cap:.0f} B/s)'
        elif global_msg_cap > 0 and g_msgs + 1 > global_msg_cap:
            reason = f'global msg-rate budget ({g_msgs + 1:.0f}/{global_msg_cap:.0f} msg/s)'
        over_budget = reason is not None
        if over_budget and ts - self._budget_drop_logged.get(topic_name, 0.0) >= 5.0:
            self._budget_drop_logged[topic_name] = ts
            self.get_logger().warning(f'[topic_data] {topic_name}: dropping ({size} bytes) - over {reason}')
        # Window boundaries roll over either way, even when dropping this
        # message — otherwise a window that opened before a cap was ever hit
        # could never advance past its 1s mark while messages keep getting
        # dropped, wedging the topic permanently. Only the running totals
        # skip the increment when the message is actually dropped.
        if over_budget:
            self._topic_byte_bucket[topic_name] = (t_start, t_bytes)
            self._global_topic_data_bucket = (g_start, g_bytes, g_msgs)
        else:
            self._topic_byte_bucket[topic_name] = (t_start, t_bytes + size)
            self._global_topic_data_bucket = (g_start, g_bytes + size, g_msgs + 1)
            # This message is actually being sent - the one point that's
            # true for every topic_data message, binary or plain JSON,
            # regardless of which of the two callers above got here. A
            # message dropped by the Hz throttle earlier in _on_topic_msg
            # never reaches this function at all, so recording only here
            # already accounts for both gates without checking either
            # explicitly.
            with self._topic_rate_lock:
                self._topic_delivered_rate_timestamps.setdefault(topic_name, deque()).append(ts)
                self._topic_delivered_byte_timestamps.setdefault(topic_name, deque()).append((ts, size))
        return over_budget

    def _resolve_topic_rate_hz(self, override: dict, msg_type: str) -> tuple[float, str]:
        """Resolves rate_hz from the per-topic override, then PointCloud2's
        own default, then the general default. Returns (rate_hz, source) - source is used only for the
        one-time log in _subscribe_to_topic, not repeated per message.
        """
        rate_hz = override.get('rate_hz')
        if rate_hz is not None:
            return rate_hz, 'per-topic override'
        if msg_type == 'sensor_msgs/msg/PointCloud2':
            return self.get_parameter('pointcloud_default_rate_hz').get_parameter_value().double_value, 'pointcloud_default_rate_hz default'
        return self.get_parameter('topic_data_rate_hz').get_parameter_value().double_value, 'topic_data_rate_hz default'

    def _publish_topic_rates(self):
        """Periodic 1Hz timer callback — recomputes every subscribed topic's
        rate_hz/bytes_per_sec (demand: sampled on receipt, before the Hz
        throttle or byte budget get a say) and delivered_rate/delivered_bytes
        (reality: sampled only for a message that actually got sent, see
        _topic_data_over_budget) from rolling windows, and pushes all four as
        one message - independent of whether that topic published anything
        this tick, which is what makes a quiet topic's numbers correctly
        decay to 0 instead of freezing at their last computed value."""
        if not self.ws or not self.loop:
            return

        with self._topic_subs_lock:
            subscribed = list(self._topic_subs.keys())

        now = time.time()
        cutoff = now - self._RATE_WINDOW_S

        def _drain(buckets: dict, topic: str, is_pair: bool):
            """Shared cutoff/average logic for all four rolling windows below
            - rates/byte_rates (demand, before either gate) and
            delivered_rates/delivered_byte_rates (after both gates) all have
            the exact same shape, just fed from a different timestamp buffer."""
            buf = buckets.get(topic)
            if not buf:
                return 0.0
            if is_pair:
                while buf and buf[0][0] < cutoff:
                    buf.popleft()
                return round(sum(size for _, size in buf) / self._RATE_WINDOW_S, 1)
            while buf and buf[0] < cutoff:
                buf.popleft()
            return round(len(buf) / self._RATE_WINDOW_S, 2)

        rates = {}
        byte_rates = {}
        delivered_rates = {}
        delivered_byte_rates = {}
        with self._topic_rate_lock:
            for topic in subscribed:
                rates[topic] = _drain(self._topic_rate_timestamps, topic, is_pair=False)
                byte_rates[topic] = _drain(self._topic_byte_timestamps, topic, is_pair=True)
                delivered_rates[topic] = _drain(self._topic_delivered_rate_timestamps, topic, is_pair=False)
                delivered_byte_rates[topic] = _drain(self._topic_delivered_byte_timestamps, topic, is_pair=True)
            # Drop buffers for topics no longer subscribed so these dicts don't
            # grow unbounded across repeated subscribe/unsubscribe cycles.
            for bucket in (
                self._topic_rate_timestamps, self._topic_byte_timestamps,
                self._topic_delivered_rate_timestamps, self._topic_delivered_byte_timestamps,
            ):
                for stale in set(bucket) - set(subscribed):
                    del bucket[stale]

        self._enqueue({
            'type': 'topic_rates',
            'rates': rates,
            'byte_rates': byte_rates,
            'delivered_rates': delivered_rates,
            'delivered_byte_rates': delivered_byte_rates,
            'timestamp': now,
        })
