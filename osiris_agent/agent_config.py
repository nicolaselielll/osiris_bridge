"""Agent configuration: resolves every setting from the params file, the cloud agent_config or its default, and applies it."""

import time

from rclpy.parameter import Parameter

from .agent_config_validation import validate_agent_config
from .bt_collector import BTCollector
from .command_policy import CommandLimits
from .qos import STATUS_QOS
from .ros2_control_collector import Ros2ControlCollector
from .tf_tree_collector import TfTreeCollector

# Settings only the robot sets, with a ROS parameter or a params file. The
# cloud configuration never carries them, so setting one on the robot doesn't
# make the rest of the cloud configuration step aside.
ROBOT_ONLY_PARAMETERS = frozenset({'bag_output_dir'})


def cloud_config_overrides(parameter_overrides):
    """The parameters set on the robot (on the command line or in a params
    file) that make the agent run from its own settings instead of the cloud
    configuration: all of them except the robot-only ones."""
    return set(parameter_overrides) - ROBOT_ONLY_PARAMETERS


class AgentConfigMixin:

    def _apply_agent_config_fallback(self):
        self._agent_config_fallback_timer.cancel()
        if self._agent_config_received:
            return
        self.get_logger().warning('No agent_config received within timeout — applying local param defaults')
        self._apply_agent_config({})

    def _resolve_config_value(self, name, config, local_default, cast=None):
        """All-or-nothing per agent run, not a per-field override: passing
        --params-file at all is a deliberate choice to run off that yaml
        file, so every field resolves from it (yaml value if the file sets
        this one, else its hardcoded declare_parameter default) and the
        cloud agent_config is ignored entirely for the whole run — not just
        for the specific fields the yaml file happens to set. Without a
        params file, every field resolves from the cloud config if present,
        else the hardcoded default. self._param_overrides being non-empty at
        all (regardless of which names it contains) is what decides which
        source every field uses — the two sources never mix within a run.
        """
        if self._param_overrides:
            return local_default
        if name in config:
            return cast(config[name]) if cast else config[name]
        return local_default

    def _apply_agent_config(self, config: dict) -> None:
        """Applies the gateway-pushed feature-toggle config (sent right after
        auth_success on every connect, and by _apply_agent_config_fallback if
        that never arrives). Precedence is all-or-nothing per run: with a
        local yaml/CLI override, every field comes from it or its hardcoded
        default and the cloud agent_config is ignored; otherwise each field
        comes from the cloud agent_config, else its hardcoded default — see
        _resolve_config_value. A robot with nothing set in agent_config, and
        no yaml override either, runs on the hardcoded defaults. Single
        dispatch point so each new toggle (Graph, Params, Nav2, Goals, BT,
        TF Tree) has one place to land rather than scattering config reads
        across the agent.

        A cloud push is validated as a whole before anything changes, and a
        push with any malformed field is rejected (see _reject_agent_config).
        With a params file the cloud config is never read, so it is not
        validated either.
        """
        if not self._param_overrides:
            config, problems = validate_agent_config(config)
            if problems:
                self._reject_agent_config(problems)
                return
        self._agent_config_received = True

        self._telemetry_enabled = self._resolve_config_value('telemetry_enabled', config, self._telemetry_enabled_default, bool)

        # TF tree: resolve the final answer, then construct/destroy the
        # collector at most once to reach it — never both in the same pass.
        tf_tree_enabled = self._resolve_config_value('tf_tree_enabled', config, self._tf_tree_enabled_default, bool)
        if tf_tree_enabled and self._tf_tree is None:
            self._tf_tree = TfTreeCollector(
                node=self,
                event_callback=self._on_tf_tree_event,
                logger=self.get_logger(),
            )
            self.get_logger().info('TF tree monitoring started')
        elif not tf_tree_enabled and self._tf_tree is not None:
            self._tf_tree.destroy()
            self._tf_tree = None
            self.get_logger().info('TF tree monitoring stopped')

        # TF tree poll timer: independent of the collector above — _poll_tf_tree
        # is a no-op whenever self._tf_tree is None, so the timer's period is
        # resolved and (re)created here regardless of tf_tree_enabled, exactly
        # like telemetry's own timer runs unconditionally.
        tf_tree_poll_interval = self._resolve_config_value('tf_tree_poll_interval', config, self._tf_tree_poll_interval_default, float)
        if self._tf_tree_poll_timer is None or self._tf_tree_poll_interval != tf_tree_poll_interval:
            if self._tf_tree_poll_timer is not None:
                self._tf_tree_poll_timer.cancel()
            self._tf_tree_poll_timer = self.create_timer(tf_tree_poll_interval, self._poll_tf_tree)
            self._tf_tree_poll_interval = tf_tree_poll_interval

        # ros2_control: same construct/destroy-at-most-once pattern as TF
        # tree, plus the same reconnect-if-changed handling as bt_conn below
        # — poll_interval is a constructor arg baked into the collector
        # (rate-limit check in Ros2ControlCollector.poll()), not re-read
        # live, so a changed interval while already enabled needs an actual
        # destroy+reconstruct to ever take effect. Its poll timer comes and
        # goes with it: a hardware component changing state, or a controller
        # switched twice within one interval, changes nothing in the ROS
        # graph, so the graph check's own poll would leave the reported state
        # stale. A period of 0 or less (possible from a params file, which
        # isn't validated) gets no timer, since it would spin.
        ros2_control_enabled = self._resolve_config_value('ros2_control_enabled', config, self._ros2_control_enabled_default, bool)
        ros2_control_poll_interval = self._resolve_config_value('ros2_control_poll_interval', config, self._ros2_control_poll_interval_default, float)
        if ros2_control_enabled and (self._ros2_control is None or self._ros2_control_poll_interval != ros2_control_poll_interval):
            if self._ros2_control is not None:
                self._ros2_control.destroy()
            self._ros2_control = Ros2ControlCollector(
                node=self,
                event_callback=self._on_ros2_control_event,
                logger=self.get_logger(),
                poll_interval=ros2_control_poll_interval,
            )
            self._ros2_control_poll_interval = ros2_control_poll_interval
            if self._ros2_control_poll_timer is not None:
                self._ros2_control_poll_timer.cancel()
            self._ros2_control_poll_timer = (
                self.create_timer(ros2_control_poll_interval, self._poll_ros2_control)
                if ros2_control_poll_interval > 0 else None
            )
            self.get_logger().info('ros2_control monitoring started')
        elif not ros2_control_enabled and self._ros2_control is not None:
            self._ros2_control.destroy()
            self._ros2_control = None
            if self._ros2_control_poll_timer is not None:
                self._ros2_control_poll_timer.cancel()
                self._ros2_control_poll_timer = None
            self.get_logger().info('ros2_control monitoring stopped')

        # Battery topic: the subscription's topic name is a constructor arg,
        # so it's (re)created here whenever the resolved topic changes.
        battery_topic = self._resolve_config_value('battery_topic', config, self._battery_topic_default)
        if self._battery_sub is None or self._battery_topic != battery_topic:
            if self._battery_sub is not None:
                self.destroy_subscription(self._battery_sub)
                self._battery_sub = None
            try:
                from sensor_msgs.msg import BatteryState as BatteryStateMsg
                self._battery_sub = self.create_subscription(
                    BatteryStateMsg, battery_topic,
                    self._on_battery_state, STATUS_QOS,
                )
                self.get_logger().info(f'Battery state subscription active on {battery_topic}')
            except Exception as e:
                self.get_logger().warning(f'Battery state monitoring unavailable: {e}')
            self._battery_topic = battery_topic

        self._battery_check_enabled = self._resolve_config_value('battery_check_enabled', config, self._battery_check_enabled_default, bool)
        self._battery_min_percent = self._resolve_config_value('battery_min_percent', config, self._battery_min_percent_default, float)

        # Dock status topic — same (re)subscribe-on-change pattern as battery
        # above, and subscribed whenever a topic is configured regardless of
        # dock_check_enabled (same as _battery_sub existing independently of
        # battery_check_enabled) — so the cache is already warm by the time
        # the operator flips the check on, instead of guaranteeing an initial
        # dock_status_unknown rejection while waiting for the first message.
        # lazily imported since opennav_docking_msgs is Nav2-docking-specific,
        # not guaranteed installed on every robot (same reasoning as
        # nav2_msgs elsewhere).
        self._dock_check_enabled = self._resolve_config_value('dock_check_enabled', config, self._dock_check_enabled_default, bool)
        dock_status_topic = self._resolve_config_value('dock_status_topic', config, self._dock_status_topic_default)
        if self._dock_status_sub is None or self._dock_status_topic != dock_status_topic:
            if self._dock_status_sub is not None:
                self.destroy_subscription(self._dock_status_sub)
                self._dock_status_sub = None
                self._last_dock_status = None
                self._last_dock_status_time = None
            if dock_status_topic:
                try:
                    from opennav_docking_msgs.msg import DockStatus
                    self._dock_status_sub = self.create_subscription(
                        DockStatus, dock_status_topic,
                        self._on_dock_status, STATUS_QOS,
                    )
                    self.get_logger().info(f'Dock status subscription active on {dock_status_topic}')
                except ImportError:
                    # Normal on a robot without Nav2 docking, and on Humble,
                    # whose opennav_docking_msgs has no DockStatus. The import
                    # is retried on each apply, so installing it later works.
                    if not self._dock_status_msg_missing_logged:
                        self._dock_status_msg_missing_logged = True
                        self.get_logger().info(
                            "Dock status is off: this robot's ROS install has no "
                            'opennav_docking_msgs DockStatus message'
                        )
                except Exception as e:
                    self.get_logger().warning(f'Dock status monitoring unavailable: {e}')
            self._dock_status_topic = dock_status_topic

        # Motion command gate. Only a literal true enables commands, so a
        # malformed value in the cloud config can never switch them on. The
        # limits are applied first, so the gate never opens with stale ones.
        limits_default = self._command_limits_default
        self._command_limits = CommandLimits(
            max_speed_mps=self._resolve_config_value('command_max_speed_mps', config, limits_default.max_speed_mps, float),
            max_distance_m=self._resolve_config_value('command_max_distance_m', config, limits_default.max_distance_m, float),
            max_angle_deg=self._resolve_config_value('command_max_angle_deg', config, limits_default.max_angle_deg, float),
            max_time_allowance_s=self._resolve_config_value('command_max_time_allowance_s', config, limits_default.max_time_allowance_s, float),
        )
        commands_enabled = self._resolve_config_value('commands_enabled', config, self._commands_enabled_default, lambda v: v is True)
        if commands_enabled != self._commands_enabled:
            self.get_logger().info(f'Robot commands {"enabled" if commands_enabled else "disabled"}')
        self._commands_enabled = commands_enabled

        # Goals: no single collector object to construct/destroy — just a set
        # of per-action subscriptions. Turning on catches up on every action
        # already known (self._active_actions, populated by the graph scan
        # regardless of this flag); turning off tears all of them down.
        goals_enabled = self._resolve_config_value('goals_enabled', config, self._goals_enabled_default, bool)
        if goals_enabled and not self._goals_enabled:
            for a in self._active_actions:
                self._subscribe_action_status(a)
                self._catch_up_action_feedback(a)
            self.get_logger().info(f'Goal tracking started ({len(self._active_actions)} action(s))')
        elif not goals_enabled and self._goals_enabled:
            for a in list(set(self._action_status_subs) | set(self._action_feedback_subs)):
                self._unsubscribe_action_monitoring(a)
            self.get_logger().info('Goal tracking stopped')
        self._goals_enabled = goals_enabled

        # Params: no collector/subscription to construct/destroy either — just
        # a cache. Turning on catches up on every node already known
        # (self._active_nodes, populated by the graph scan regardless of this
        # flag); turning off clears the cache so stale values don't keep
        # showing in the Params pane after the user asked this to stop.
        params_enabled = self._resolve_config_value('params_enabled', config, self._params_enabled_default, bool)
        if params_enabled and not self._params_enabled:
            for fqn in self._active_nodes:
                self._fetch_node_parameters_async(fqn)
            self.get_logger().info(f'Param fetching started ({len(self._active_nodes)} node(s))')
        elif not params_enabled and self._params_enabled:
            self._node_parameter_cache.clear()
            self._pending_param_fetches.clear()
            self._param_fetch_changes.clear()
            self._graph_dirty = True
            self.get_logger().info('Param fetching stopped')
        self._params_enabled = params_enabled

        # BT: mutually exclusive by construction — Nav2 BT and BT.CPP share
        # the same event pipeline (_on_bt_event/_cached_bt_tree_event, no
        # source tagging), so only one may ever be active. Tear down
        # whichever isn't the resolved mode before starting the other, so a
        # switch never briefly has both running. BT.CPP additionally
        # reconnects if its host/port changed while already in btcpp mode —
        # a running BTCollector is a live ZMQ connection bound to whatever
        # host/port it was constructed with.
        bt_mode = self._resolve_config_value('bt_mode', config, self._bt_mode_default)
        bt_host = self._resolve_config_value('bt_host', config, self._bt_host_default)
        bt_server_port = self._resolve_config_value('bt_server_port', config, self._bt_server_port_default, int)
        bt_publisher_port = self._resolve_config_value('bt_publisher_port', config, self._bt_publisher_port_default, int)
        bt_conn = (bt_host, bt_server_port, bt_publisher_port)

        if bt_mode != 'nav2' and self._nav2_bt_monitor_initialized:
            self._teardown_nav2_bt_monitor()
        if bt_mode != 'btcpp' and self._bt_collector is not None:
            self._bt_collector.stop()
            self._bt_collector = None
            self._bt_collector_conn = None

        if bt_mode == 'nav2' and not self._nav2_bt_monitor_initialized:
            self._init_nav2_bt_monitor()
        elif bt_mode == 'btcpp' and (self._bt_collector is None or self._bt_collector_conn != bt_conn):
            if self._bt_collector is not None:
                self._bt_collector.stop()
            self._bt_collector = BTCollector(
                event_callback=self._on_bt_event,
                host=bt_host,
                server_port=bt_server_port,
                publisher_port=bt_publisher_port,
                logger=self.get_logger(),
            )
            self._bt_collector.start()
            self._bt_collector_conn = bt_conn
            self.get_logger().info(f'BT.CPP monitoring started ({bt_host}:{bt_server_port}/{bt_publisher_port})')

        self._bt_mode = bt_mode

        # graph_debounce_interval / topic_data_rate_hz /
        # topic_data_max_bytes_per_sec / global_topic_data_max_bytes_per_sec /
        # global_topic_data_max_msgs_per_sec / image_jpeg_quality /
        # image_max_dimension / pointcloud_max_points /
        # pointcloud_keep_intensity / pointcloud_keep_rgb /
        # pointcloud_default_rate_hz: every consumer already calls
        # self.get_parameter(...) fresh at time of use (a plain path string
        # re-read on each bag list/download/record; a plain float re-read
        # into a brand-new threading.Timer on every debounce trigger, not a
        # fixed recurring ROS timer; a plain float/int re-read on every topic
        # message in _on_topic_msg/_topic_data_over_budget) — so there's
        # nothing to construct or defer here, just update the underlying ROS
        # param when overridden. Same all-or-nothing rule as
        # _resolve_config_value: a yaml params file being present at all
        # (regardless of whether it sets these specific fields) means the
        # cloud config is skipped for all of them — the ROS param already holds
        # the yaml-or-hardcoded value and is simply left untouched.
        # bag_output_dir isn't among them: it's set on the robot only (see
        # ROBOT_ONLY_PARAMETERS), since it bounds what a bag download may read.
        if not self._param_overrides and 'graph_debounce_interval' in config:
            self.set_parameters([Parameter('graph_debounce_interval', Parameter.Type.DOUBLE, float(config['graph_debounce_interval']))])
        if not self._param_overrides and 'topic_data_rate_hz' in config:
            self.set_parameters([Parameter('topic_data_rate_hz', Parameter.Type.DOUBLE, float(config['topic_data_rate_hz']))])
        if not self._param_overrides and 'topic_data_max_bytes_per_sec' in config:
            self.set_parameters([Parameter('topic_data_max_bytes_per_sec', Parameter.Type.DOUBLE, float(config['topic_data_max_bytes_per_sec']))])
        if not self._param_overrides and 'global_topic_data_max_bytes_per_sec' in config:
            self.set_parameters([Parameter('global_topic_data_max_bytes_per_sec', Parameter.Type.DOUBLE, float(config['global_topic_data_max_bytes_per_sec']))])
        if not self._param_overrides and 'global_topic_data_max_msgs_per_sec' in config:
            self.set_parameters([Parameter('global_topic_data_max_msgs_per_sec', Parameter.Type.DOUBLE, float(config['global_topic_data_max_msgs_per_sec']))])
        if not self._param_overrides and 'image_jpeg_quality' in config:
            self.set_parameters([Parameter('image_jpeg_quality', Parameter.Type.INTEGER, int(config['image_jpeg_quality']))])
        if not self._param_overrides and 'image_max_dimension' in config:
            self.set_parameters([Parameter('image_max_dimension', Parameter.Type.INTEGER, int(config['image_max_dimension']))])
        if not self._param_overrides and 'pointcloud_max_points' in config:
            self.set_parameters([Parameter('pointcloud_max_points', Parameter.Type.INTEGER, int(config['pointcloud_max_points']))])
        if not self._param_overrides and 'pointcloud_keep_intensity' in config:
            self.set_parameters([Parameter('pointcloud_keep_intensity', Parameter.Type.BOOL, bool(config['pointcloud_keep_intensity']))])
        if not self._param_overrides and 'pointcloud_keep_rgb' in config:
            self.set_parameters([Parameter('pointcloud_keep_rgb', Parameter.Type.BOOL, bool(config['pointcloud_keep_rgb']))])
        if not self._param_overrides and 'pointcloud_default_rate_hz' in config:
            self.set_parameters([Parameter('pointcloud_default_rate_hz', Parameter.Type.DOUBLE, float(config['pointcloud_default_rate_hz']))])

        # topic_limits: per-topic overrides of the defaults above (rate_hz,
        # max_bytes_per_sec, jpeg_quality, max_dimension and the pointcloud_*
        # keys in TOPIC_LIMIT_FIELDS; see _topic_limit_overrides,
        # _on_topic_msg, _topic_data_over_budget). Not a set_parameters call
        # like the scalars above — ROS2 declared parameters are a flat,
        # known-in-advance namespace, a poor fit for a dynamic topic-name-
        # keyed map — so this is plain instance state instead, consulted
        # fresh on every topic message exactly like the scalar params are.
        # Same all-or-nothing yaml-override rule: a params-file run clears
        # it rather than leaving stale overrides from a previous cloud push.
        with self._topic_limit_lock:
            if self._param_overrides:
                self._topic_limit_overrides = {}
            elif 'topic_limits' in config:
                self._topic_limit_overrides = dict(config['topic_limits'] or {})
            # Snapshot for the resolved_agent_config ground-truth send below —
            # taken under the same lock rather than re-read there unguarded.
            _topic_limits_snapshot = dict(self._topic_limit_overrides)

        # Deliberately includes every topic_data, image and point cloud field,
        # not just the toggles, so a live push that only touches e.g.
        # image_jpeg_quality still shows up in the log instead of looking like
        # a no-op call. Read fresh off the ROS params (not local vars) so this
        # is accurate regardless of which fields this particular call
        # actually touched.
        self.get_logger().info(
            f'Applied agent_config: telemetry_enabled={self._telemetry_enabled}, '
            f'tf_tree_enabled={tf_tree_enabled}, goals_enabled={goals_enabled}, '
            f'params_enabled={params_enabled}, bt_mode={bt_mode}, '
            f'commands_enabled={commands_enabled}, '
            f'topic_data_rate_hz={self.get_parameter("topic_data_rate_hz").get_parameter_value().double_value:.1f}, '
            f'topic_data_max_bytes_per_sec={self.get_parameter("topic_data_max_bytes_per_sec").get_parameter_value().double_value:.0f}, '
            f'global_topic_data_max_bytes_per_sec={self.get_parameter("global_topic_data_max_bytes_per_sec").get_parameter_value().double_value:.0f}, '
            f'global_topic_data_max_msgs_per_sec={self.get_parameter("global_topic_data_max_msgs_per_sec").get_parameter_value().double_value:.0f}, '
            f'image_jpeg_quality={self.get_parameter("image_jpeg_quality").get_parameter_value().integer_value}, '
            f'image_max_dimension={self.get_parameter("image_max_dimension").get_parameter_value().integer_value}, '
            f'pointcloud_max_points={self.get_parameter("pointcloud_max_points").get_parameter_value().integer_value}, '
            f'pointcloud_keep_intensity={self.get_parameter("pointcloud_keep_intensity").get_parameter_value().bool_value}, '
            f'pointcloud_keep_rgb={self.get_parameter("pointcloud_keep_rgb").get_parameter_value().bool_value}, '
            f'pointcloud_default_rate_hz={self.get_parameter("pointcloud_default_rate_hz").get_parameter_value().double_value:.1f}, '
            f'topic_limits={_topic_limits_snapshot}'
        )

        # Ground truth for the client: what this agent is ACTUALLY running
        # with right now, for every field — as opposed to the cloud
        # agent_config, which can be completely irrelevant (yaml-file runs
        # ignore it outright) or simply stale until this agent reconnects.
        # Sent on every _apply_agent_config call (every connect/reconnect,
        # and the 5s fallback), so the client always has a current answer,
        # never a guess based on pane data being empty. The fields pushed
        # through set_parameters() above (bag_output_dir,
        # graph_debounce_interval and the topic_data, image and point cloud
        # settings) are read fresh off the ROS params here rather than from
        # local vars, since that is the one place their resolved value lives.
        #
        # yaml_override is metadata about the SOURCE these values were
        # resolved from, not a config value itself — a yaml/CLI params file
        # being passed at all means the client shouldn't claim a restart
        # will sync Cloud Config (it never will, as long as that file keeps
        # getting passed); it should instead tell the operator how to
        # actually switch back to cloud-driven config.
        self._resolved_agent_config = {
            'telemetry_enabled': self._telemetry_enabled,
            'goals_enabled': goals_enabled,
            'params_enabled': params_enabled,
            'tf_tree_enabled': tf_tree_enabled,
            'tf_tree_poll_interval': tf_tree_poll_interval,
            'ros2_control_enabled': ros2_control_enabled,
            'ros2_control_poll_interval': ros2_control_poll_interval,
            'battery_topic': battery_topic,
            'battery_check_enabled': self._battery_check_enabled,
            'battery_min_percent': self._battery_min_percent,
            'dock_check_enabled': self._dock_check_enabled,
            'dock_status_topic': dock_status_topic,
            'commands_enabled': commands_enabled,
            'command_max_speed_mps': self._command_limits.max_speed_mps,
            'command_max_distance_m': self._command_limits.max_distance_m,
            'command_max_angle_deg': self._command_limits.max_angle_deg,
            'command_max_time_allowance_s': self._command_limits.max_time_allowance_s,
            'bt_mode': bt_mode,
            'bt_host': bt_host,
            'bt_server_port': bt_server_port,
            'bt_publisher_port': bt_publisher_port,
            'bag_output_dir': self.get_parameter('bag_output_dir').get_parameter_value().string_value,
            'graph_debounce_interval': self.get_parameter('graph_debounce_interval').get_parameter_value().double_value,
            'topic_data_rate_hz': self.get_parameter('topic_data_rate_hz').get_parameter_value().double_value,
            'topic_data_max_bytes_per_sec': self.get_parameter('topic_data_max_bytes_per_sec').get_parameter_value().double_value,
            'global_topic_data_max_bytes_per_sec': self.get_parameter('global_topic_data_max_bytes_per_sec').get_parameter_value().double_value,
            'global_topic_data_max_msgs_per_sec': self.get_parameter('global_topic_data_max_msgs_per_sec').get_parameter_value().double_value,
            'image_jpeg_quality': self.get_parameter('image_jpeg_quality').get_parameter_value().integer_value,
            'image_max_dimension': self.get_parameter('image_max_dimension').get_parameter_value().integer_value,
            'pointcloud_max_points': self.get_parameter('pointcloud_max_points').get_parameter_value().integer_value,
            'pointcloud_keep_intensity': self.get_parameter('pointcloud_keep_intensity').get_parameter_value().bool_value,
            'pointcloud_keep_rgb': self.get_parameter('pointcloud_keep_rgb').get_parameter_value().bool_value,
            'pointcloud_default_rate_hz': self.get_parameter('pointcloud_default_rate_hz').get_parameter_value().double_value,
            'topic_limits': _topic_limits_snapshot,
        }
        self._send_resolved_agent_config()

    def _send_resolved_agent_config(self):
        self._enqueue({
            'type': 'resolved_agent_config',
            'yaml_override': bool(self._param_overrides),
            'config': self._resolved_agent_config,
            'timestamp': time.time(),
        })

    def _reject_agent_config(self, problems):
        """Ignore a malformed push as a whole: every setting keeps its value.
        Motion commands are the exception and get switched off, since the
        rejected push may have been the one turning them off, and an agent
        that can't read its config should not keep accepting motion. They
        stay off until a valid push arrives."""
        self.get_logger().error(f'Ignored agent_config, keeping the current settings: {"; ".join(problems)}')
        if not self._commands_enabled:
            return
        self._commands_enabled = False
        self.get_logger().warning('Robot commands disabled until a valid agent_config arrives')
        if self._resolved_agent_config is not None:
            self._resolved_agent_config = {**self._resolved_agent_config, 'commands_enabled': False}
            self._send_resolved_agent_config()
