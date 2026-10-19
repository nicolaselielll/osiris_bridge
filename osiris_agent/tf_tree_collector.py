"""
Collector for the TF frame tree.

Uses a tf2_ros.Buffer, fed from wherever the robot publishes TF (see
tf_listener.py), to build a structured snapshot of all known TF frames:
parent/child hierarchy, transform values, age, and staleness.
Called from the graph check and from its own poll timer
(tf_tree_poll_interval), with a rate limit and a change-detection guard so it
only emits when something meaningful changes.
"""

import time

from .tf_listener import TfListener, resolve_tf_namespace, tf_namespace_published


# Staleness threshold in seconds — a frame not updated in this window is marked stale.
DEFAULT_STALE_THRESHOLD = 1.0

# How often (seconds) to actually query the buffer — 10 Hz = 0.1 s.
POLL_INTERVAL = 0.1


class TfTreeCollector:
    """Polls tf2 buffer for the full frame tree and emits tf_tree events on change."""

    # tf2_ros may not be installed — import is deferred and the collector
    # degrades gracefully when the package is absent.
    _tf2_available = None  # None = not checked yet

    def __init__(self, node, event_callback, logger,
                 stale_threshold=DEFAULT_STALE_THRESHOLD):
        self._node = node
        self._event_callback = event_callback
        self._logger = logger
        self._stale_threshold = stale_threshold

        self._tf_buffer = None
        self._tf_listener = None

        # Change-detection cache: last emitted snapshot dict
        self._last_sent_snapshot = None
        # Structural key for change detection (excludes age_seconds/stamp)
        self._last_structural_key = None

        # Rate-limiting: skip poll if called too soon
        self._last_poll_time = 0.0

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def poll(self, force: bool = False):
        """Check TF tree state and emit a tf_tree event on structural change.

        Args:
            force: Skip the rate-limit guard (used by initial_state to ensure a
                   fresh snapshot is available regardless of when poll() last ran).
        """
        now = time.time()
        if not force and now - self._last_poll_time < POLL_INTERVAL:
            return
        self._last_poll_time = now

        if not self._ensure_tf2():
            return

        if not self._ensure_listener():
            return

        snapshot = self._build_frame_tree()
        if snapshot is None:
            return

        # Emit empty snapshot when all frames disappear (e.g. nav2 stopped)
        if not snapshot and self._last_sent_snapshot:
            self._last_sent_snapshot = {}
            self._event_callback({'type': 'tf_tree', 'timestamp': now, 'data': {}})
            return

        # Change detection: compare only structural parts (ignore age_seconds/stamp
        # which change every poll even when the robot is stationary).
        structural = self._structural_key(snapshot)
        if structural != self._last_structural_key:
            self._last_structural_key = structural
            self._last_sent_snapshot = snapshot
            self._event_callback({
                'type': 'tf_tree',
                'timestamp': now,
                'data': snapshot,
            })

    def get_snapshot(self):
        """Return last known TF tree snapshot (for initial_state)."""
        return self._last_sent_snapshot

    def follow_published_tf(self):
        """Switch to another namespace's TF when the one followed is no longer
        published and another is: a Nav2 brought up under a namespace, or later
        than this agent. Called from the graph check."""
        if self._tf_listener is None or tf_namespace_published(self._node, self._tf_listener.namespace):
            return
        namespace = resolve_tf_namespace(self._node)
        if namespace != self._tf_listener.namespace:
            self._logger.info(f"TfTreeCollector: reading TF from '{namespace or '/'}'")
            self.destroy()
            self._ensure_listener(namespace)

    def destroy(self):
        """Release the TF listener's subscriptions."""
        if self._tf_listener is not None:
            self._tf_listener.unregister()
        self._tf_listener = None
        self._tf_buffer = None

    # ------------------------------------------------------------------
    # Internals
    # ------------------------------------------------------------------

    @staticmethod
    def _structural_key(snapshot):
        """Return a hashable key covering only structural aspects of the snapshot.

        Excludes 'age_seconds' and 'stamp' which change every poll cycle
        regardless of whether anything meaningful has changed.
        """
        key = {}
        for frame_id, entry in snapshot.items():
            key[frame_id] = (
                entry.get('parent'),
                tuple(sorted(entry.get('children', []))),
                entry.get('is_stale'),
                entry.get('is_static'),
                # Round transform to 3 dp so minor float jitter doesn't trigger events
                tuple(
                    round(v, 3)
                    for v in (
                        entry.get('transform') or {}
                    ).get('translation', {}).values()
                ),
                tuple(
                    round(v, 3)
                    for v in (
                        entry.get('transform') or {}
                    ).get('rotation', {}).values()
                ),
            )
        return frozenset(key.items())

    def _ensure_tf2(self):
        """Lazily check whether tf2_ros is importable."""
        if TfTreeCollector._tf2_available is not None:
            return TfTreeCollector._tf2_available

        try:
            import tf2_ros  # noqa: F401
            TfTreeCollector._tf2_available = True
        except ImportError:
            self._logger.info(
                "tf2_ros not found – tf_tree panel disabled"
            )
            TfTreeCollector._tf2_available = False

        return TfTreeCollector._tf2_available

    def _ensure_listener(self, namespace=None):
        """Create the Buffer and its TF listener on first successful call."""
        if self._tf_listener is not None:
            return True
        try:
            import tf2_ros
            if namespace is None:
                namespace = resolve_tf_namespace(self._node)
            self._tf_buffer = tf2_ros.Buffer()
            self._tf_listener = TfListener(self._tf_buffer, self._node, namespace)
            return True
        except Exception as e:
            self._logger.warning(f"TfTreeCollector: could not create listener: {e}")
            return False

    def _build_frame_tree(self):
        """
        Enumerate all frames in the buffer and build a hierarchy dict.

        Returns a dict keyed by frame_id:
        {
          "odom": {
            "parent": "map",
            "children": ["base_link"],
            "transform": {
              "translation": {"x": ..., "y": ..., "z": ...},
              "rotation":    {"x": ..., "y": ..., "z": ..., "w": ...}
            },
            "stamp": <float seconds>,
            "age_seconds": <float>,
            "is_stale": <bool>,
            "is_static": <bool>
          },
          ...
        }
        The root frame (no parent in the buffer) has "parent": null. A static
        transform (from /tf_static) has no time: it holds until replaced, so it
        and the root frame are never stale and have no age.
        """
        import yaml
        from rclpy.time import Time as RclpyTime

        # Get list of all known frame IDs
        try:
            frame_ids = self._tf_buffer._getFrameStrings()
        except Exception as e:
            self._logger.debug(f"TfTreeCollector: _getFrameStrings failed: {e}")
            return None

        if not frame_ids:
            return {}

        # Parse YAML to get parent info per frame
        parent_map = {}
        try:
            yaml_str = self._tf_buffer.all_frames_as_yaml()
            frames_yaml = yaml.safe_load(yaml_str)
            if isinstance(frames_yaml, dict):
                for frame_id, info in frames_yaml.items():
                    if isinstance(info, dict):
                        parent = info.get('parent') or None
                        if parent:
                            parent_map[frame_id] = parent
        except Exception as e:
            self._logger.debug(f"TfTreeCollector: could not parse frames YAML: {e}")

        now = time.time()
        result = {}

        for frame_id in frame_ids:
            parent = parent_map.get(frame_id)

            # Try to look up the transform from parent → frame
            stamp = None
            translation = None
            rotation = None
            is_static = False
            if parent:
                try:
                    t = self._tf_buffer.lookup_transform(
                        parent,
                        frame_id,
                        RclpyTime(),  # latest available
                    )
                    tr = t.transform.translation
                    ro = t.transform.rotation
                    translation = {'x': tr.x, 'y': tr.y, 'z': tr.z}
                    rotation = {'x': ro.x, 'y': ro.y, 'z': ro.z, 'w': ro.w}
                    stamp_msg = t.header.stamp
                    if stamp_msg.sec == 0 and stamp_msg.nanosec == 0:
                        is_static = True
                    else:
                        stamp = stamp_msg.sec + stamp_msg.nanosec * 1e-9
                except Exception:
                    pass

            age_seconds = (now - stamp) if stamp is not None else None
            if is_static or not parent:
                is_stale = False
            else:
                is_stale = (age_seconds is None or age_seconds > self._stale_threshold)

            result[frame_id] = {
                'parent': parent,
                'children': [],        # populated in second pass below
                'transform': {
                    'translation': translation,
                    'rotation': rotation,
                } if (translation is not None) else None,
                'stamp': stamp,
                'age_seconds': round(age_seconds, 3) if age_seconds is not None else None,
                'is_stale': is_stale,
                'is_static': is_static,
            }

        # Second pass: populate children lists
        for frame_id, entry in result.items():
            parent = entry['parent']
            if parent and parent in result:
                result[parent]['children'].append(frame_id)

        # Sort children for stable change-detection
        for entry in result.values():
            entry['children'].sort()

        return result
