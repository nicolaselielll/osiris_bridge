"""Payload encoding: binary framing of byte-array topics, JPEG re-encoding of raw images and point cloud downsampling."""

import io
import math

# Pillow re-encodes raw sensor_msgs/Image frames to JPEG before they hit the
# binary-framing path (see _reencode_image_jpeg) - a real install_requires
# dependency (setup.py), but imported defensively rather than unconditionally:
# an agent updated via `git pull` without a matching `pip install -e .` would
# otherwise hard-crash at startup over what's really just a bandwidth nicety.
# Missing Pillow just means Image topics send raw pixel bytes.
try:
    from PIL import Image as PILImage
    _PIL_AVAILABLE = True
except ImportError:
    _PIL_AVAILABLE = False

# ROS2 message types whose payload is dominated by one large byte array. For
# these, _on_topic_msg strips that field out of the JSON topic_data message
# and sends it as a separate raw binary WS frame instead — see _on_topic_msg
# and BINARY_MARKER_KEY. message_to_ordereddict + json.dumps would otherwise
# turn e.g. a 640x480 rgb8 Image's ~920KB data field into a JSON array of
# ints (commas, digit characters, no base64 even) — 3-6x the raw byte count,
# and by far the single biggest source of avoidable bytes on the agent's
# uplink. All four types below happen to name the field 'data'; the field for
# OccupancyGrid is int8 (needs sign-aware decoding on the way back, see
# BINARY_MARKER_KEY), the other three are uint8.
BINARY_PAYLOAD_FIELD = 'data'
BINARY_PAYLOAD_TYPES = {
    'sensor_msgs/msg/Image':          False,  # uint8[]
    'sensor_msgs/msg/CompressedImage': False,  # uint8[]
    'sensor_msgs/msg/PointCloud2':    False,  # uint8[]
    'nav_msgs/msg/OccupancyGrid':     True,   # int8[] — signed, see BINARY_MARKER_KEY
}
# Marker substituted for BINARY_PAYLOAD_FIELD in the JSON header — must match
# the key the gateway looks for exactly, it's the two ends of one wire
# protocol. 'len' is the byte count the binary frame right behind this
# header is expected to carry (gateway-side sanity check only); 'signed' says
# whether those bytes are int8 (two's-complement, e.g. OccupancyGrid's -1
# "unknown" cells) or plain uint8.
BINARY_MARKER_KEY = '__osiris_binary__'

# Hard, non-overridable ceiling on pointcloud_max_points (see
# _resolve_pointcloud_max_points) - a last-resort safety valve against a
# typo or runaway driver, not a constraint on reasonable per-topic choices.
# At ~16 bytes/point (xyz+intensity float32), 300k points is ~4.8MB/message
# - at a real sensor's publish rate this is real sustained memory/bandwidth
# pressure on the agent (~48MB/s at 10Hz), so the ceiling sits there rather
# than higher.
POINTCLOUD_MAX_POINTS_HARD_CEILING = 300_000

# sensor_msgs/Image encodings _reencode_image_jpeg knows how to decode -
# (PIL mode, PIL raw-decoder rawmode) pairs. PIL's 'raw' decoder accepts
# rawmode directly as e.g. 'BGR'/'BGRA' and does the channel reorder itself,
# so bgr8/bgra8 (by far the most common OpenCV-sourced encodings) need no
# manual byte-swapping - just the right rawmode string. Depth encodings
# (16UC1, 32FC1, ...) and bayer/YUV formats aren't listed; those topics are
# sent as raw bytes.
IMAGE_ENCODING_PIL_MODES = {
    'rgb8':  ('RGB', 'RGB'),
    'bgr8':  ('RGB', 'BGR'),
    'mono8': ('L', 'L'),
    'rgba8': ('RGBA', 'RGBA'),
    'bgra8': ('RGBA', 'BGRA'),
}
_IMAGE_ENCODING_CHANNELS = {'L': 1, 'RGB': 3, 'BGR': 3, 'RGBA': 4, 'BGRA': 4}

# sensor_msgs/msg/PointField.datatype -> byte size. Fixed by the ROS message
# definition itself (INT8=1 ... FLOAT64=8), not something a driver can
# redefine, so this table never needs updating per-robot the way the image
# encoding one above sometimes might.
POINTFIELD_DATATYPE_SIZES = {1: 1, 2: 1, 3: 2, 4: 2, 5: 4, 6: 4, 7: 4, 8: 8}


class PayloadEncodingMixin:

    def _reencode_image_jpeg(self, raw: bytes, width, height, encoding, quality, max_dimension, topic_name: str):
        """Re-encodes a raw sensor_msgs/Image byte buffer to JPEG via Pillow
        - a raw rgb8/bgr8 frame is close to the worst case possible per byte
        actually sent, so this is the real bandwidth win for a live camera
        feed (the binary framing above just removes JSON's own encoding
        overhead, it doesn't touch the pixel data's size at all).

        Returns (jpeg_bytes, out_width, out_height) on success, None if
        Pillow isn't installed, the encoding isn't one of the common ones in
        IMAGE_ENCODING_PIL_MODES (a depth/bayer/YUV camera), or the buffer's
        length doesn't match what width/height/encoding declare (a corrupt
        or partial frame) - callers fall back to sending the original raw
        bytes. Never raises; a
        genuine encode failure is caught and treated the same as an
        unsupported encoding.
        """
        if not _PIL_AVAILABLE or not width or not height:
            return None
        modes = IMAGE_ENCODING_PIL_MODES.get(encoding)
        if modes is None:
            if topic_name not in self._image_reencode_unsupported_warned:
                self._image_reencode_unsupported_warned.add(topic_name)
                self.get_logger().info(
                    f"[image] {topic_name}: encoding '{encoding}' not supported for JPEG "
                    f"re-encode, sending raw pixel bytes"
                )
            return None
        pil_mode, raw_mode = modes
        expected_len = width * height * _IMAGE_ENCODING_CHANNELS[raw_mode]
        if len(raw) != expected_len:
            return None
        try:
            img = PILImage.frombuffer(pil_mode, (width, height), raw, 'raw', raw_mode, 0, 1)
            if pil_mode == 'RGBA':
                # JPEG has no alpha channel - drop it rather than fail the
                # whole re-encode over a channel nothing downstream reads yet.
                img = img.convert('RGB')
            if max_dimension and max(img.width, img.height) > max_dimension:
                scale = max_dimension / max(img.width, img.height)
                new_size = (max(1, round(img.width * scale)), max(1, round(img.height * scale)))
                img = img.resize(new_size, PILImage.BILINEAR)
            buf = io.BytesIO()
            img.save(buf, format='JPEG', quality=quality)
            return buf.getvalue(), img.width, img.height
        except Exception as e:
            self.get_logger().warning(f'[image] {topic_name}: JPEG re-encode failed: {e}')
            return None

    def _resolve_pointcloud_max_points(self, override: dict = None) -> int:
        """Resolves pointcloud_max_points from the per-topic override, else
        the global parameter, then clamps it to
        POINTCLOUD_MAX_POINTS_HARD_CEILING regardless of source - protects
        the agent process itself (payload build cost, WS frame size) from an
        extreme value, before any consumer applies its own limits. Not a
        declare_parameter on purpose - a hard ceiling isn't meant to be
        user-adjustable at all, unlike pointcloud_max_points itself.
        """
        max_points = override.get('pointcloud_max_points') if override else None
        if max_points is None:
            max_points = self.get_parameter('pointcloud_max_points').get_parameter_value().integer_value
        return min(max_points, POINTCLOUD_MAX_POINTS_HARD_CEILING)

    def _downsample_pointcloud(self, raw: bytes, data: dict, max_points: int, keep_intensity: bool, keep_rgb: bool, topic_name: str):
        """Trims a PointCloud2's per-point byte layout down to xyz (plus
        intensity/rgb if requested and actually present on this cloud), and
        subsamples by uniform stride if there are more than max_points.

        Pure byte-slicing, no numeric decoding. A point's fields are already
        packed as fixed-width byte ranges (PointField name/offset/datatype/
        count) - keeping a field just means copying its exact byte range
        verbatim into the rebuilt buffer at a new offset. Endianness and
        what a field's bytes actually mean are never touched, so this works
        identically regardless of is_bigendian or the concrete numbers
        involved - the same reasoning _on_topic_msg's `& 0xFF` byte
        extraction in topic_bridge.py already relies on.

        Returns (new_data_bytes, new_fields, new_point_step, new_point_count)
        on success, or None if x/y/z aren't all present, the buffer doesn't
        match what width/height/point_step declare (a corrupt/partial
        cloud), or the result would be empty. Never raises - a failure here
        falls through to sending the original, untrimmed cloud, same
        fallback behavior _reencode_image_jpeg's own None return gets.
        """
        try:
            fields = data.get('fields') or []
            point_step = data.get('point_step')
            width = data.get('width') or 0
            height = data.get('height') or 1
            if not point_step or not width:
                return None

            by_name = {f.get('name'): f for f in fields if f.get('name')}
            if not all(n in by_name for n in ('x', 'y', 'z')):
                return None
            keep_names = ['x', 'y', 'z']
            if keep_intensity and 'intensity' in by_name:
                keep_names.append('intensity')
            if keep_rgb:
                # Only one of these is ever present on a real cloud - rgb
                # (packed float) is the more common convention, rgba the
                # less common one some drivers use instead.
                if 'rgb' in by_name:
                    keep_names.append('rgb')
                elif 'rgba' in by_name:
                    keep_names.append('rgba')

            kept = []
            for name in keep_names:
                f = by_name[name]
                unit_size = POINTFIELD_DATATYPE_SIZES.get(f.get('datatype'))
                offset = f.get('offset')
                if unit_size is None or offset is None:
                    return None
                kept.append((name, offset, unit_size * max(1, f.get('count') or 1)))

            total_points = width * height
            if len(raw) < point_step * total_points:
                return None

            new_point_step = sum(size for _, _, size in kept)
            if new_point_step <= 0:
                return None

            stride = max(1, math.ceil(total_points / max_points)) if max_points and max_points > 0 else 1
            out = bytearray()
            for i in range(0, total_points, stride):
                base = i * point_step
                for _, offset, size in kept:
                    out += raw[base + offset: base + offset + size]

            out_points = len(out) // new_point_step
            if out_points == 0:
                return None

            new_fields = []
            running_offset = 0
            for name, _, size in kept:
                orig = by_name[name]
                new_fields.append({
                    'name': name,
                    'offset': running_offset,
                    'datatype': orig.get('datatype'),
                    'count': orig.get('count') or 1,
                })
                running_offset += size

            return bytes(out), new_fields, new_point_step, out_points
        except Exception as e:
            self.get_logger().warning(f'[pointcloud] {topic_name}: downsample failed: {e}')
            return None
