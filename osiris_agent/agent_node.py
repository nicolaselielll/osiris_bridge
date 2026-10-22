"""ROS 2 node that bridges a robot to the Osiris gateway over WebSocket.

WebBridge is assembled from one mixin per job, each in its own module
(connection, agent_config, graph_monitor, topic_bridge, commands, ...).
The mixins share the state that WebBridge.__init__ sets up, which is also
where every ROS parameter the agent declares is listed with its default.
"""

import asyncio
import os
import signal
import subprocess
import sys
import threading
import time
from collections import deque

import psutil
import rclpy

from rcl_interfaces.msg import ParameterEvent
from rclpy.node import Node
from rclpy.qos import qos_profile_parameter_events
from std_msgs.msg import Empty as EmptyMsg

from osiris_agent import __version__ as AGENT_VERSION
from .action_monitor import ActionMonitorMixin
from .agent_config import AgentConfigMixin, cloud_config_overrides
from .bag_recorder import BagRecorderMixin
from .command_policy import (
    DEFAULT_MAX_ANGLE_DEG,
    DEFAULT_MAX_DISTANCE_M,
    DEFAULT_MAX_SPEED_MPS,
    DEFAULT_MAX_TIME_ALLOWANCE_S,
    CommandLimits,
)
from .commands import CommandMixin
from .connection import ConnectionMixin
from .graph_monitor import WATCHER_CHECK_S, GraphMonitorMixin
from .graph_watcher_launcher import forward_stderr, start_graph_watcher
from .lifecycle import LifecycleMixin
from .nav2_bt_monitor import Nav2BtMonitorMixin
from .node_parameters import NodeParametersMixin
from .payload_encoding import PayloadEncodingMixin
from .snapshots import SnapshotMixin
from .telemetry import TELEMETRY_INTERVAL, TelemetryMixin
from .topic_bridge import TopicBridgeMixin


class WebBridge(
    ConnectionMixin,
    AgentConfigMixin,
    GraphMonitorMixin,
    NodeParametersMixin,
    LifecycleMixin,
    ActionMonitorMixin,
    TopicBridgeMixin,
    PayloadEncodingMixin,
    SnapshotMixin,
    BagRecorderMixin,
    CommandMixin,
    TelemetryMixin,
    Nav2BtMonitorMixin,
    Node,
):

    def __init__(self, watcher_proc=None):
        super().__init__('osiris_node')
        self._watcher_proc = watcher_proc

        # Names explicitly set via --params-file/CLI at launch — distinct
        # from self.get_parameter(name), which can't tell "the user
        # explicitly passed this" apart from "nothing was passed, this is
        # just the hardcoded declare_parameter default." _apply_agent_config
        # needs that distinction to give a deliberate local override
        # precedence over the cloud agent_config (see _resolve_config_value).
        # Node.get_parameter_overrides() isn't available on this rclpy
        # version — self._parameter_overrides is the underlying dict rclpy
        # itself populates from --params-file/CLI before any
        # declare_parameter() call consumes it, and is what that method
        # would have wrapped anyway. A robot-only setting such as
        # bag_output_dir doesn't count: the cloud never sets it.
        self._param_overrides = cloud_config_overrides(self._parameter_overrides.keys())

        auth_token = os.environ.get('OSIRIS_AUTH_TOKEN')
        if not auth_token:
            raise ValueError("OSIRIS_AUTH_TOKEN environment variable must be set")

        # Declare tunable parameters
        self.declare_parameter('telemetry_enabled',      True)
        self.declare_parameter('goals_enabled',          True)
        self.declare_parameter('params_enabled',         True)
        self.declare_parameter('tf_tree_enabled',        False)
        self.declare_parameter('ros2_control_enabled',        False)
        self.declare_parameter('ros2_control_poll_interval',    2.0)
        self.declare_parameter('battery_topic',          '/battery_state')
        # Toggle + value, same split as tf_tree_enabled/tf_tree_poll_interval
        # and ros2_control_enabled/ros2_control_poll_interval above — not a
        # 0/empty sentinel on the value itself, so a real threshold/topic can
        # be kept configured while toggled off, and so there's an actual
        # switch in the UI rather than "clear the field to disable it". Both
        # default off — forcing either on by default would start rejecting
        # drive/spin on any robot whose battery topic doesn't report a usable
        # percentage, or that has no dock at all. See
        # _check_battery_and_dock_preconditions.
        self.declare_parameter('battery_check_enabled',  False)
        self.declare_parameter('battery_min_percent',    30.0)
        self.declare_parameter('dock_check_enabled',     False)
        self.declare_parameter('dock_status_topic',      '/dock_status')
        # Motion commands (spin/drive) are rejected unless commands_enabled is
        # true, and every accepted command must stay within the limits below.
        # Cancelling a running command is always allowed. See command_policy.py.
        self.declare_parameter('commands_enabled',             False)
        self.declare_parameter('command_max_speed_mps',        DEFAULT_MAX_SPEED_MPS)
        self.declare_parameter('command_max_distance_m',       DEFAULT_MAX_DISTANCE_M)
        self.declare_parameter('command_max_angle_deg',        DEFAULT_MAX_ANGLE_DEG)
        self.declare_parameter('command_max_time_allowance_s', DEFAULT_MAX_TIME_ALLOWANCE_S)
        # Nav2 BT and BT.CPP share a single event pipeline
        # (_on_bt_event/_cached_bt_tree_event, no source tagging), so both
        # being active at once would corrupt each other's state. One
        # three-way setting makes that mutual exclusion structural instead of
        # a UI convention to enforce. Default 'nav2': Nav2 BT monitoring on,
        # BT.CPP off.
        self.declare_parameter('bt_mode',               'nav2')  # 'off' | 'nav2' | 'btcpp'
        self.declare_parameter('bt_host',               '127.0.0.1')
        self.declare_parameter('bt_server_port',         1667)
        self.declare_parameter('bt_publisher_port',      1668)
        self.declare_parameter('tf_tree_poll_interval',   0.2)
        self.declare_parameter('graph_debounce_interval',   1.0)
        self.declare_parameter('bag_output_dir',            '~/ros2_bags')
        # Max rate (Hz) topic_data messages get forwarded at, per topic; 0
        # disables the throttle entirely. See _on_topic_msg. This is the
        # DEFAULT applied to every topic - a specific topic in
        # self._topic_limit_overrides (set live via agent_config's
        # 'topic_limits', see _apply_agent_config) takes precedence over it.
        self.declare_parameter('topic_data_rate_hz',         50.0)
        # Same default-with-per-topic-override relationship as
        # topic_data_rate_hz above, just bytes/sec instead of messages/sec —
        # this is what actually protects against a single heavy topic (a
        # pointcloud, an uncompressed image) rather than just a chatty one. 0
        # disables it. See _topic_data_over_budget.
        self.declare_parameter('topic_data_max_bytes_per_sec', 0.0)
        # Connection-wide budgets, shared across every subscribed topic - the
        # actual constraint being protected is the agent's one uplink, which
        # no per-topic cap alone can protect (five topics each individually
        # "within budget" can still saturate the link combined). Only
        # topic_data competes for these; graph state, lifecycle events, etc.
        # go through plain _enqueue and are never throttled here on purpose,
        # so control-plane traffic never starves because the sensor stream is
        # busy. Both 0 = unlimited. See _topic_data_over_budget.
        self.declare_parameter('global_topic_data_max_bytes_per_sec', 0.0)
        self.declare_parameter('global_topic_data_max_msgs_per_sec',  0.0)
        # sensor_msgs/Image only (CompressedImage is already compressed by
        # the camera driver, left untouched) - re-encoded to JPEG before
        # binary framing, see _reencode_image_jpeg. Unlike the caps above,
        # these default ON rather than to a disabled/0 state: raw
        # uncompressed pixel bytes are close to worst case for a live feed,
        # so a sane default (unlike an opt-in cap) makes the agent better
        # out of the box, not just capable of being tuned. Same default-
        # with-per-topic-override relationship as topic_data_rate_hz - an
        # override in self._topic_limit_overrides ('jpeg_quality'/
        # 'max_dimension') takes precedence per topic.
        self.declare_parameter('image_jpeg_quality', 70)     # 1-95, JPEG quality
        self.declare_parameter('image_max_dimension', 640)   # long edge, px; 0 = never resize
        # sensor_msgs/PointCloud2 only - see _downsample_pointcloud. Same
        # default-on, default-with-per-topic-override shape as the image
        # params above (topic_limits keys: 'pointcloud_max_points'/
        # 'pointcloud_keep_intensity'/'pointcloud_keep_rgb'). Trims to xyz
        # only and caps the point count by default to keep large point
        # clouds from consuming excess bandwidth/storage.
        self.declare_parameter('pointcloud_max_points', 10000)
        self.declare_parameter('pointcloud_keep_intensity', False)
        self.declare_parameter('pointcloud_keep_rgb', False)
        # PointCloud2-specific fallback for the Hz throttle below, used
        # instead of the general topic_data_rate_hz default when a
        # PointCloud2 topic has no per-topic rate_hz override - a large
        # point cloud costs far more per message than what that general
        # default was sized for.
        self.declare_parameter('pointcloud_default_rate_hz', 1.0)

        base_url = os.environ.get('OSIRIS_WS_URL', 'wss://osiris-gateway.fly.dev')
        self.ws_url = f'{base_url}?robot=true&token={auth_token}'

        self.ws   = None
        self.loop = None
        self._send_queue: asyncio.Queue | None = None

        # ── Bag recording ─────────────────────────────────────────────────────
        self._bag_proc: subprocess.Popen | None = None
        self._bag_output_path: str | None = None
        self._bag_lock = threading.Lock()

        # ── Topic subscriptions (gateway-requested) ──────────────────────────
        self._topic_subs: dict[str, rclpy.subscription.Subscription] = {}
        self._topic_subs_lock = threading.Lock()
        # topic -> resolved ROS2 type string, set alongside _topic_subs. Lets
        # _on_topic_msg decide whether this topic's messages get the binary
        # WS-frame treatment (see BINARY_PAYLOAD_TYPES) without re-querying
        # the graph on every single message.
        self._topic_msg_types: dict[str, str] = {}
        # topic -> qos_key() of the QoS its bridge subscription uses; see
        # _refresh_bridge_qos.
        self._topic_sub_qos: dict[str, tuple] = {}
        # Rolling window of receipt timestamps per subscribed topic, used to
        # recompute each topic's rate_hz on a 1Hz timer (see _publish_topic_rates)
        # rather than piggybacking a value on topic_data itself — a piggybacked
        # rate only updates when a new message arrives, so it freezes at its last
        # value instead of decaying toward zero once a topic goes quiet.
        self._topic_rate_timestamps: dict[str, deque] = {}
        # Same idea as _topic_rate_timestamps, (ts, size) pairs instead of
        # bare timestamps - the bytes/sec counterpart shown next to rate_hz
        # in the UI (see _publish_topic_rates). Deliberately separate from
        # _topic_byte_bucket below: that one is a 1s enforcement bucket that
        # resets on a hard boundary (fine for a threshold check, useless for
        # display - read at the wrong instant it'd show a stale zero right
        # after a reset even at full rate). Same lock/window as
        # _topic_rate_timestamps since both only ever feed that one
        # 1Hz report.
        self._topic_byte_timestamps: dict[str, deque] = {}
        # Same two rolling windows again, but recorded only for a message
        # that actually gets sent (see _topic_data_over_budget's own
        # not-over-budget branch, the one place both the Hz throttle above
        # and the byte budget have already been survived) - the "what's
        # really going out" counterpart to _topic_rate_timestamps/
        # _topic_byte_timestamps above, which measure demand before either
        # gate. Shown in the UI as "source -> delivered" rather than the
        # configured limit, since delivered can genuinely sit anywhere under
        # a byte cap depending on how frames happen to size that second, so
        # the limit alone doesn't say what's actually arriving.
        self._topic_delivered_rate_timestamps: dict[str, deque] = {}
        self._topic_delivered_byte_timestamps: dict[str, deque] = {}
        self._topic_rate_lock = threading.Lock()
        self._RATE_WINDOW_S = 5.0
        # Last-forwarded time per topic, used to cap how often topic_data
        # actually gets serialized and sent (see _on_topic_msg) — independent
        # of _topic_rate_timestamps above, which keeps sampling every real
        # receipt so rate_hz stays accurate regardless of this throttle.
        self._topic_data_throttle: dict[str, float] = {}
        # topic -> (window_start_ts, bytes_forwarded_in_window), and the
        # connection-wide equivalent right below — both only ever touched
        # from _on_topic_msg, which (like _topic_data_throttle above) only
        # ever runs on the single ROS executor thread, so neither needs a
        # lock. See _topic_data_over_budget.
        self._topic_byte_bucket: dict[str, tuple[float, int]] = {}
        self._global_topic_data_bucket: tuple[float, int, int] = (0.0, 0, 0)  # (window_start_ts, bytes, msgs)
        # topic -> per-topic overrides (rate_hz, max_bytes_per_sec,
        # jpeg_quality, max_dimension and the pointcloud_* keys in
        # TOPIC_LIMIT_FIELDS) of the defaults above, set live from
        # agent_config's 'topic_limits' (see _apply_agent_config). Written from the asyncio-loop thread
        # (_apply_agent_config, via _receive_loop) and read from the ROS
        # executor thread (_on_topic_msg) — genuinely cross-thread, unlike
        # the two buckets above, hence the lock (same reasoning as
        # _topic_subs_lock guarding _topic_msg_types).
        self._topic_limit_overrides: dict[str, dict] = {}
        self._topic_limit_lock = threading.Lock()
        # Topics whose Image encoding _reencode_image_jpeg doesn't recognize
        # (a depth/bayer/YUV camera, say) - warned once per topic rather than
        # once per frame, since the encoding is a fixed property of the
        # topic and will never suddenly become supported mid-stream.
        self._image_reencode_unsupported_warned: set[str] = set()
        # topic -> last time _topic_data_over_budget logged a drop for it,
        # throttled to once per 5s per topic rather than once per dropped
        # frame (which, under a tight budget, could be every single frame).
        # The Hz throttle in _on_topic_msg has no equivalent - what rate a
        # topic is capped at is logged once, in _subscribe_to_topic, when
        # the subscription starts, since being throttled afterward is the
        # normal continuous state for a capped topic, not a per-drop event.
        self._budget_drop_logged: dict[str, float] = {}

        # ── Existence caches (set of fully-qualified names) ───────────────────
        self._active_nodes:    set[str] = set()
        self._active_topics:   set[str] = set()
        self._active_services: dict[str, str] = {}
        self._active_actions:  set[str] = set()

        # ── Action type cache ─────────────────────────────────────────────────
        # None  = fetch attempted but failed (package not installed, etc.)
        # dict  = { goal_type, result_type, feedback_type,
        #           goal_fields, result_fields, feedback_fields }
        self._action_type_cache: dict[str, dict | None] = {}

        # ── Action monitoring (status + feedback) ─────────────────────────────
        self._action_status_subs:       dict[str, rclpy.subscription.Subscription] = {}
        self._action_feedback_subs:     dict[str, rclpy.subscription.Subscription] = {}
        self._action_goal_states:       dict[str, dict[str, int]] = {}  # action → {uuid_hex → status_int}
        self._action_feedback_throttle: dict[str, float] = {}           # action → last_sent_time
        self._action_status_unread:     set[str] = set()                # actions whose status subscription has had no message yet
        # None = not yet resolved by _apply_agent_config — the graph scan
        # still runs and populates _active_actions (Graph's action listing
        # needs that regardless), but _subscribe_action_status is skipped
        # until this becomes True, same "don't do the work just to undo it"
        # reasoning as TF tree. Once resolved, toggling calls _subscribe_
        # action_status/_unsubscribe_action_monitoring for every action
        # already known in _active_actions to catch up/tear down.
        self._goals_enabled_default = self.get_parameter('goals_enabled').get_parameter_value().bool_value
        self._goals_enabled = None

        # ── Count sentinels (cheap change detection) ─────────────────────────
        self._topic_counts: dict[str, tuple[int, int]] = {}  # topic → (pub_n, sub_n)

        # ── Relation caches (populated by Tier-2 enrichment) ─────────────────
        self._topic_relations: dict[str, dict] = {}

        # ── Enrichment pending queues ─────────────────────────────────────────
        self._pending_topic_enrichment: set[str] = set()

        # ── Parameters (lazy-loaded, async) ──────────────────────────────────
        self._node_parameter_cache: dict[str, dict | None] = {}  # None = not yet fetched, {} = fetched but empty
        self._pending_param_fetches: set[str] = set()
        self._nodes_no_param_service: set[str] = set()  # nodes whose list_parameters was never ready
        self._param_fetch_changes: dict[str, dict] = {}  # node_fqn → parameter changes seen during its fetch
        # None = not yet resolved by _apply_agent_config. New nodes' params are
        # only discovered through the graph tick, so if graph monitoring could
        # ever be switched off, this would only cover the nodes that existed at
        # the first tick. Graph monitoring has no separate toggle.
        self._params_enabled_default = self.get_parameter('params_enabled').get_parameter_value().bool_value
        self._params_enabled = None

        # ── Lifecycle subscriptions (auto-detected managed nodes) ─────────────
        self._lifecycle_subs: dict[str, rclpy.subscription.Subscription] = {}  # topic → sub
        self._lifecycle_state_cache: dict[str, str] = {}   # node_fqn → state label
        self._pending_lifecycle_fetches: set[str] = set()  # node_fqns with in-flight get_state calls

        # ── Command sending (send_command_request / cancel_command_request) ───
        # Fixed enum of known Nav2 actions the agent will send goals to —
        # deliberately not a generic "call any action" capability, so what a
        # remote command can make the robot do stays bounded. Goal PROGRESS/OUTCOME is not
        # re-reported here — it already flows through the existing generic
        # _on_action_status/_on_action_feedback pipeline (goal_event etc.),
        # which monitors every action in the graph regardless of who sent the
        # goal. This section only handles: precondition check, sending,
        # accept/reject, and the manual-cancel channel.
        self._command_action_clients: dict[tuple, object] = {}  # (command, action name) → ActionClient
        self._active_command_lock = threading.Lock()
        self._active_command_pending = False       # True from claim until accept/reject is known
        self._active_command_goal_handle = None     # set once accepted, cleared on terminal/cancel
        self._active_command_timeout_timer = None   # force-releases the slot if the goal never
                                                      # produces a result at all (see _on_command_
                                                      # send_response / _force_release_active_command)

        # ── Snapshot & dirty-flag ─────────────────────────────────────────────
        self._last_sent_nodes:    dict | None = None
        self._last_sent_topics:   dict | None = None
        self._last_sent_actions:  dict | None = None
        self._last_sent_services: dict | None = None
        self._graph_dirty = False
        self._graph_debounce_timer: threading.Timer | None = None
        self._graph_check_pending = False  # set when a trigger arrives while a check is already running

        # ── Service scan throttle ─────────────────────────────────────────────
        self._service_rescan_ticks: int = 0

        # ── Initial scan synchronization ──────────────────────────────────────
        self._initial_scan_complete = threading.Event()
        self._first_graph_check_done = False
        self._graph_check_lock = threading.Lock()  # serializes concurrent _check_graph_changes calls
        self._param_fetch_timer = None  # one-shot timer for delayed initial param fetch
        # When the agent started, and when its view of the graph last changed:
        # the first connect waits for discovery to settle (_wait_for_graph_to_settle).
        self._started_at = time.monotonic()
        self._graph_seen = None
        self._graph_changed_at = self._started_at
        # ── BT state ─────────────────────────────────────────────────────────
        self._cached_bt_tree_event: dict | None = None

        # ── Telemetry ─────────────────────────────────────────────────────────
        self._last_disk_io      = None
        self._last_net_io       = None
        self._last_io_time:     float | None = None
        self._last_battery_state: dict | None = None
        self._last_battery_state_time: float | None = None
        self._telemetry_tick = 0  # counts periodic samples — see TELEMETRY_PROCESS_EVERY_N_TICKS
        psutil.cpu_percent(interval=None)  # prime — first call always returns 0.0
        # Starts at the local ROS param (yaml value if the user set one, else
        # its declared default of True) — overridden by the gateway's
        # agent_config push once connected, see _apply_agent_config. If that
        # push never arrives (offline, gateway didn't have a config saved,
        # etc.) this local value is simply never overwritten, so the yaml/
        # default value stands. The timer itself always runs; this just gates
        # whether each tick actually sends anything, so toggling it takes
        # effect on the very next tick with no timer start/stop bookkeeping.
        self._telemetry_enabled_default = self.get_parameter('telemetry_enabled').get_parameter_value().bool_value
        self._telemetry_enabled = self._telemetry_enabled_default

        # ── Collectors ────────────────────────────────────────────────────────
        # Not constructed here — same reasoning as TF Tree below: building a
        # Ros2ControlCollector now from the local param and possibly tearing
        # it straight back down once agent_config arrives would be wasted
        # work. Left as None until _apply_agent_config resolves the final
        # answer (config override, else this local default) and constructs
        # it at most once. self._ros2_control is checked for None everywhere
        # it's read.
        self._ros2_control_enabled_default = self.get_parameter('ros2_control_enabled').get_parameter_value().bool_value
        self._ros2_control_poll_interval_default = self.get_parameter('ros2_control_poll_interval').get_parameter_value().double_value
        self._ros2_control = None
        self._ros2_control_poll_interval = None
        self._ros2_control_poll_timer = None
        # Not constructed here — TfTreeCollector isn't free (real TF buffer/
        # listener), so building it now from the local param and possibly
        # tearing it straight down once agent_config arrives would be wasted
        # work. Left as None until _apply_agent_config resolves the final
        # answer (config override, else this local default) and constructs
        # it at most once. self._tf_tree is checked for None everywhere it's
        # read.
        self._tf_tree_enabled_default = self.get_parameter('tf_tree_enabled').get_parameter_value().bool_value
        self._tf_tree = None

        # ── Timers ────────────────────────────────────────────────────────────
        # Subscribe to C++ graph watcher events — event-driven polls.
        # The one-shot startup timer guarantees an initial scan even when the
        # C++ watcher binary is unavailable (e.g. pip install without binary).
        # Use VOLATILE (default) depth=10: the startup timer handles initial state,
        # and live events are reliably received without TRANSIENT_LOCAL replay
        # which would race with the 1s startup timer on a background thread.
        self.create_subscription(
            EmptyMsg, '/osiris/graph_changed',
            self._on_graph_changed, 10,
        )
        # Nodes publish one event per parameter they declare, so a Nav2 bring-up
        # sends several hundred at once; this profile keeps the last 1000.
        self.create_subscription(
            ParameterEvent, '/parameter_events',
            self._on_parameter_event, qos_profile_parameter_events,
        )
        self._startup_check_timer = self.create_timer(1.0, self._do_startup_check)
        self.create_timer(WATCHER_CHECK_S, self._check_graph_watcher)
        self.create_timer(TELEMETRY_INTERVAL,          self._collect_telemetry)
        self.create_timer(1.0,                         self._publish_topic_rates)
        # Not created here — its period is a constructor arg to create_timer,
        # so like TF Tree/ros2_control above it's deferred to
        # _apply_agent_config, which resolves config override vs. this local
        # default and creates the timer at most once per resolved interval.
        self._tf_tree_poll_interval_default = self.get_parameter('tf_tree_poll_interval').get_parameter_value().double_value
        self._tf_tree_poll_timer = None
        self._tf_tree_poll_interval = None

        # One-time safety net: if the gateway never delivers agent_config (no
        # network, gateway down, whatever) within a few seconds of startup,
        # resolve every toggle to its local param default rather than leaving
        # things like TF tree stuck at None forever. Not needed on later
        # reconnects — once a real config has arrived once, this is a no-op.
        # Only relevant when actually waiting on the cloud in the first
        # place — a yaml params file means there's nothing to wait for (see
        # the immediate _apply_agent_config({}) call at the end of __init__),
        # so no fallback timer is created in that case either.
        self._agent_config_received = False
        # What the agent last reported running with, see _apply_agent_config.
        self._resolved_agent_config = None
        self._agent_config_fallback_timer = None if self._param_overrides else self.create_timer(5.0, self._apply_agent_config_fallback)

        # ── Battery state subscription ────────────────────────────────────────
        # Not subscribed here — the topic name is a constructor arg to
        # create_subscription, so like the timers above it's deferred to
        # _apply_agent_config, which resolves config override vs. this local
        # default and (re)subscribes at most once per resolved topic.
        self._battery_topic_default = self.get_parameter('battery_topic').get_parameter_value().string_value
        self._battery_sub = None
        self._battery_topic = None
        self._battery_check_enabled_default = self.get_parameter('battery_check_enabled').get_parameter_value().bool_value
        self._battery_check_enabled = self._battery_check_enabled_default
        self._battery_min_percent_default = self.get_parameter('battery_min_percent').get_parameter_value().double_value
        self._battery_min_percent = self._battery_min_percent_default

        # ── Motion command gate ───────────────────────────────────────────────
        # Commands stay disabled, with the default limits, until
        # _apply_agent_config resolves the real values, so a command arriving
        # before the first config is applied is rejected rather than accepted.
        self._commands_enabled_default = self.get_parameter('commands_enabled').get_parameter_value().bool_value
        self._commands_enabled = False
        self._command_limits_default = CommandLimits(
            max_speed_mps=self.get_parameter('command_max_speed_mps').get_parameter_value().double_value,
            max_distance_m=self.get_parameter('command_max_distance_m').get_parameter_value().double_value,
            max_angle_deg=self.get_parameter('command_max_angle_deg').get_parameter_value().double_value,
            max_time_allowance_s=self.get_parameter('command_max_time_allowance_s').get_parameter_value().double_value,
        )
        self._command_limits = CommandLimits()

        # ── Dock status subscription ────────────────────────────────────────
        # Same deferred-construction reasoning as battery above — topic name
        # is a constructor arg, resolved once by _apply_agent_config.
        self._dock_check_enabled_default = self.get_parameter('dock_check_enabled').get_parameter_value().bool_value
        self._dock_check_enabled = self._dock_check_enabled_default
        self._dock_status_topic_default = self.get_parameter('dock_status_topic').get_parameter_value().string_value
        self._dock_status_sub = None
        self._dock_status_topic = None
        # Set once the missing DockStatus message has been logged, so the
        # retry on each later config apply stays quiet.
        self._dock_status_msg_missing_logged = False
        self._last_dock_status: dict | None = None
        self._last_dock_status_time: float | None = None

        # ── WebSocket thread ──────────────────────────────────────────────────
        threading.Thread(target=self._run_ws_client, daemon=True).start()

        # ── Optional BT collectors ────────────────────────────────────────────
        # Neither constructed here — resolved once by _apply_agent_config
        # (config override, else these local defaults), same deferred
        # reasoning as TF tree/goals/params. self._bt_mode = None means "not
        # yet resolved"; self._nav2_bt_monitor_initialized tells code
        # throughout the agent whether the Nav2 BT monitor has been set up,
        # since that isn't guaranteed to have happened by the time any of that
        # code runs.
        self._bt_mode_default = self.get_parameter('bt_mode').get_parameter_value().string_value
        self._bt_host_default = self.get_parameter('bt_host').get_parameter_value().string_value
        self._bt_server_port_default = self.get_parameter('bt_server_port').get_parameter_value().integer_value
        self._bt_publisher_port_default = self.get_parameter('bt_publisher_port').get_parameter_value().integer_value
        self._bt_mode = None
        self._bt_collector = None
        self._bt_collector_conn = None  # (host, server_port, publisher_port) BTCollector is currently connected with
        self._nav2_bt_monitor_initialized = False
        # The Nav2 being followed; set by _init_nav2_bt_monitor (see
        # _resolve_nav2_namespace), root names until then.
        self._nav2_bt_log_topic = '/behavior_tree_log'
        self._nav2_bt_navigator = '/bt_navigator'

        # A yaml params file means the operator already fully specified how
        # this run should behave — there's nothing to wait on the cloud for,
        # so resolve and construct everything right now instead of leaving
        # every feature at None until a WS connection (or the 5s fallback
        # timer, not even created in this case) gets around to it. Passing
        # {} as config makes _resolve_config_value fall through to each
        # field's local_default unconditionally, same as it always would
        # once self._param_overrides is non-empty.
        if self._param_overrides:
            self._apply_agent_config({})

        _watcher_status = (
            f'pid={watcher_proc.pid}' if watcher_proc is not None else 'not started'
        )
        self.get_logger().info(
            f"🚀 Osiris agent v{AGENT_VERSION} — event based graph monitoring "
            f"(graph_watcher {_watcher_status})"
        )

    # ──────────────────────────────────────────────
    # Collector event handlers
    # ──────────────────────────────────────────────

    def _poll_tf_tree(self):
        """Timer callback (every tf_tree_poll_interval) to keep tf_tree updates flowing."""
        if self._tf_tree is not None:
            self._tf_tree.poll()

    def _poll_ros2_control(self):
        """Timer callback (every ros2_control_poll_interval): controller and
        hardware states can change without any change to the ROS graph."""
        collector = self._ros2_control  # can be swapped by _apply_agent_config meanwhile
        if collector is not None:
            collector.poll(force=True)

    def _on_ros2_control_event(self, event: dict):
        self._enqueue(event)

    def _on_tf_tree_event(self, event: dict):
        self._enqueue(event)

    # ──────────────────────────────────────────────
    # Cleanup
    # ──────────────────────────────────────────────

    def _forward_watcher_stderr(self, proc):
        """Forwards a started graph_watcher's stderr to the ROS logger, so its
        crashes are visible, falling back to plain stderr once rclpy is
        shutting down to avoid publishing on an invalid context.
        forward_stderr keeps reading whatever a line holds, so the pipe never
        fills and blocks graph_watcher."""
        if not proc or not proc.stderr:
            return
        ros_log = self.get_logger()

        def _log_watcher_line(line):
            if rclpy.ok():
                try:
                    ros_log.info(f'[gw] {line}')
                    return
                except Exception:
                    pass
            print(f'[gw] {line}', file=sys.stderr, flush=True)
        threading.Thread(target=forward_stderr, args=(proc.stderr, _log_watcher_line), daemon=True).start()

    def destroy_node(self):
        # Cancel pending debounce timer so it doesn't fire after shutdown.
        if self._graph_debounce_timer is not None:
            try:
                self._graph_debounce_timer.cancel()
            except Exception:
                pass
            self._graph_debounce_timer = None
        if self._tf_tree is not None:
            self._tf_tree.destroy()
        if self._bt_collector:
            self._bt_collector.stop()
        with self._bag_lock:
            bag_proc = self._bag_proc
            self._bag_proc = None
            self._bag_output_path = None
        if bag_proc is not None and bag_proc.poll() is None:
            try:
                bag_proc.send_signal(signal.SIGINT)
                bag_proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                bag_proc.kill()
                try:
                    bag_proc.wait(timeout=2)
                except subprocess.TimeoutExpired:
                    pass
        super().destroy_node()


def main(args=None):
    _watcher_proc = start_graph_watcher()

    rclpy.init(args=args)
    node = WebBridge(watcher_proc=_watcher_proc)
    node._forward_watcher_stderr(_watcher_proc)

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    except Exception:
        # A Ctrl+C can land while the executor builds its wait set, which then
        # fails with an RCLError because the context is shutting down. That's
        # a normal stop; any error while ROS is still running is real.
        if rclpy.ok():
            raise
    finally:
        # Stop the C++ watcher first so its stderr pipe closes and the
        # forwarder thread exits before we tear down the rclpy context. The
        # node's own: a restarted watcher replaced the one started here.
        _watcher_proc = node._watcher_proc
        if _watcher_proc is not None:
            try:
                _watcher_proc.terminate()
                _watcher_proc.wait(timeout=3)
            except subprocess.TimeoutExpired:
                _watcher_proc.kill()
                try:
                    _watcher_proc.wait(timeout=2)
                except Exception:
                    pass
            except Exception:
                pass
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            try:
                rclpy.shutdown()
            except Exception:
                pass


if __name__ == '__main__':
    main()
    