"""Graph monitoring: detects ROS graph changes, enriches topic relations and sends node, topic, action and service snapshots."""

import threading
import time

from std_msgs.msg import Empty as EmptyMsg

from .graph_watcher_launcher import start_graph_watcher

# Services to suppress from graph output (internal ROS2 plumbing)
_SUPPRESSED_SERVICE_PREFIXES = ('/ros2cli_daemon',)

# Right after the agent starts, DDS discovery is still finding the robot's
# nodes. The agent connects once no node or topic has changed for
# GRAPH_SETTLE_S, and at the latest GRAPH_SETTLE_MAX_S after it started. A
# participant missed in discovery's first burst shows up at its next
# announcement, every 3 s by default with Fast DDS, so the quiet time has to
# be longer than that.
GRAPH_SETTLE_S = 4.0
GRAPH_SETTLE_MAX_S = 15.0

# A graph watcher that exits is started again: its process is checked every
# WATCHER_CHECK_S. After WATCHER_QUICK_RESTARTS restarts within
# WATCHER_RESTART_WINDOW_S, the next waits WATCHER_RESTART_WINDOW_S, so one
# that keeps crashing doesn't flood the log.
WATCHER_CHECK_S = 5.0
WATCHER_QUICK_RESTARTS = 3
WATCHER_RESTART_WINDOW_S = 60.0


class GraphMonitorMixin:

    # ──────────────────────────────────────────────
    # Tier-1: cheap existence detection
    # ──────────────────────────────────────────────

    def _check_graph_changes(self):
        if not self._graph_check_lock.acquire(blocking=False):
            # A check is already running (e.g. slow introspection during
            # heavy graph churn). Whatever triggered this call must not be
            # silently dropped — flag it so the in-flight run loops back
            # around and re-checks before releasing the lock. This keeps
            # the pipeline purely event-driven (no periodic polling) while
            # guaranteeing every trigger is eventually acted on.
            self._graph_check_pending = True
            return
        try:
            self._check_graph_changes_locked()
            while self._graph_check_pending:
                self._graph_check_pending = False
                self._check_graph_changes_locked()
        finally:
            self._graph_check_lock.release()

    def _check_graph_changes_locked(self):
        # ── 1. Node + topic queries (always, both cheap) ──────────────────────
        _t0 = time.time()
        node_pairs      = list(self.get_node_names_and_namespaces())
        topic_type_list = self.get_topic_names_and_types()
        _t1 = time.time()

        # Build a flat map for O(1) type lookup throughout this method
        topic_type_map = {t: types for t, types in topic_type_list}

        current_nodes   = {self._node_full_name(n, ns) for n, ns in node_pairs}
        current_topics  = {t for t, _ in topic_type_list}
        current_actions = {
            t.replace('/_action/status', '')
            for t in current_topics
            if t.endswith('/_action/status') and not self._only_own_status_subscription(t)
        }
        self.get_logger().info(
            f"[poll] node+topic: {(_t1-_t0)*1000:.1f}ms "
            f"({len(current_nodes)} nodes, {len(current_topics)} topics, {len(current_actions)} actions)"
        )
        self._note_graph(current_nodes, current_topics)

        # ── 1b. Service scan ─── on node changes and follow-up ticks only ────────
        _nodes_stopped  = self._first_graph_check_done and bool(self._active_nodes - current_nodes)
        _nodes_started  = self._first_graph_check_done and bool(current_nodes - self._active_nodes)
        _do_service_scan = (
            not self._first_graph_check_done
            or _nodes_stopped
            or _nodes_started
            or self._service_rescan_ticks > 0
        )
        if _do_service_scan:
            if _nodes_stopped:
                # Schedule follow-up scans to catch DDS endpoint lag.
                self._service_rescan_ticks = 4
            elif self._service_rescan_ticks > 0:
                self._service_rescan_ticks -= 1
            _ts0 = time.time()
            service_type_list = self.get_service_names_and_types()
            _ts1 = time.time()
            current_services = {
                s: types[0] if types else 'unknown'
                for s, types in service_type_list
                if not any(s.startswith(p) for p in _SUPPRESSED_SERVICE_PREFIXES)
            }
            self.get_logger().info(
                f"[poll] service_scan: {(_ts1-_ts0)*1000:.1f}ms ({len(current_services)} services)"
            )
        else:
            current_services = self._active_services

        # ── FIRST TICK: silently populate caches, no events ───────────────────
        if not self._first_graph_check_done:
            self._first_graph_check_done = True
            self._active_nodes    = current_nodes
            self._active_topics   = current_topics
            self._active_services = current_services
            self._active_actions  = current_actions
            _te0 = time.time()
            self._do_full_initial_enrichment(topic_type_list, node_pairs)
            _te1 = time.time()
            # Stagger parameter fetches: fire them on the ROS executor thread
            # 5 s after the first tick so we don't hammer lifecycle nodes that
            # are still in the middle of configuring/activating.
            def _fetch_all_params_delayed():
                for fqn in list(current_nodes):
                    if self._params_enabled:
                        self._fetch_node_parameters_async(fqn)
                    self._fetch_lifecycle_state_async(fqn)
            self._param_fetch_timer = self.create_timer(5.0, lambda: (self._cancel_param_fetch_timer(), _fetch_all_params_delayed()))
            for _t in current_topics:
                if _t.endswith('/transition_event'):
                    self._subscribe_lifecycle_topic(_t)
                    self._fetch_lifecycle_state_async(_t[:-len('/transition_event')])
            # Resolve action types for all actions at startup — Graph's action
            # listing needs this regardless of goals_enabled. Status
            # subscription (actual goal tracking) is gated separately; see
            # self._goals_enabled.
            for a in current_actions:
                self._fetch_action_types(a, topic_type_map)
                if self._goals_enabled:
                    self._subscribe_action_status(a)
            if self._ros2_control is not None:
                self._ros2_control.poll(force=True)
            if self._tf_tree is not None:
                self._tf_tree.poll(force=True)
            self._initial_scan_complete.set()
            self.get_logger().info(
                f"[poll] first tick complete: {len(current_nodes)} nodes, {len(current_topics)} topics, "
                f"{len(current_services)} services, {len(current_actions)} actions — "
                f"node+topic={(_t1-_t0)*1000:.1f}ms enrichment={(_te1-_te0)*1000:.1f}ms"
            )
            return

        # ── 2. Node events ────────────────────────────────────────────────────
        started_nodes = current_nodes - self._active_nodes
        if started_nodes:
            self.get_logger().info(f"[poll] {len(started_nodes)} node(s) started: {sorted(started_nodes)}")
            self._pending_topic_enrichment.update(self._active_topics)
            self._graph_dirty = True
        for fqn in started_nodes:
            self._nodes_no_param_service.discard(fqn)  # allow retry after restart
            if self._params_enabled:
                self._fetch_node_parameters_async(fqn)
            self._fetch_lifecycle_state_async(fqn)

        stopped_nodes = self._active_nodes - current_nodes
        if stopped_nodes:
            self.get_logger().info(f"[poll] {len(stopped_nodes)} node(s) stopped: {sorted(stopped_nodes)}")
            self._graph_dirty = True
        for fqn in stopped_nodes:
            for topic, rel in self._topic_relations.items():
                if fqn in rel.get('publishers', set()) or fqn in rel.get('subscribers', set()):
                    self._pending_topic_enrichment.add(topic)
            self._node_parameter_cache.pop(fqn, None)
            self._pending_param_fetches.discard(fqn)
            self._param_fetch_changes.pop(fqn, None)
            self._lifecycle_state_cache.pop(fqn, None)
            self._pending_lifecycle_fetches.discard(fqn)

        # ── 3. Topic events ───────────────────────────────────────────────────
        for t in current_topics - self._active_topics:
            self._pending_topic_enrichment.add(t)
            self._graph_dirty = True
            if t.endswith('/transition_event'):
                self._subscribe_lifecycle_topic(t)
            # Nav2 BT edge-case: the followed behavior tree log just appeared
            if t == self._nav2_bt_log_topic and self._nav2_bt_monitor_initialized:
                if self.count_publishers(t) > 0:
                    self._nav2_bt_publisher_active = True
                    if self._load_and_parse_bt_xml():
                        self._on_bt_event({
                            'type': 'bt_tree', 'timestamp': time.time(),
                            'tree_id': self._nav2_bt_tree_id,
                            'tree': self._nav2_bt_tree_structure,
                            'nodes': [{**nd, 'status': 'IDLE'} for nd in self._nav2_bt_nodes_list],
                        })

        for t in self._active_topics - current_topics:
            self._topic_relations.pop(t, None)
            self._topic_counts.pop(t, None)
            self._pending_topic_enrichment.discard(t)
            self._graph_dirty = True
            if t.endswith('/transition_event'):
                lc_sub = self._lifecycle_subs.pop(t, None)
                if lc_sub:
                    self.destroy_subscription(lc_sub)
            if t == self._nav2_bt_log_topic and self._nav2_bt_monitor_initialized:
                self._on_nav2_bt_gone()

        # ── 4. Service changes ─────────────────────────────────────────────────────────────
        if _do_service_scan:
            if set(current_services) != set(self._active_services):
                self._graph_dirty = True

        # ── 5. Action events ──────────────────────────────────────────────────
        for a in current_actions - self._active_actions:
            self._fetch_action_types(a, topic_type_map)
            if self._goals_enabled:
                self._subscribe_action_status(a)
            self._graph_dirty = True

        for a in self._active_actions - current_actions:
            self._action_type_cache.pop(a, None)
            self._unsubscribe_action_monitoring(a)
            self._graph_dirty = True

        # ── 6. Update existence caches ────────────────────────────────────────
        self._active_nodes    = current_nodes
        self._active_topics   = current_topics
        if _do_service_scan:
            self._active_services = current_services
        self._active_actions  = current_actions

        # ── 7. Re-enrich only topics whose pub/sub count changed ─────────────
        # A running node can add or drop a publisher or subscription on a topic
        # that already exists (Nav2 does in on_configure), and so can this
        # agent; no node or topic appears for that, so the counts are compared
        # on every poll. They are cheap to read, unlike the endpoint lists.
        for t in current_topics:
            counts = self._topic_counts.get(t)
            if counts is not None and counts != (self.count_publishers(t), self.count_subscribers(t)):
                self._pending_topic_enrichment.add(t)
        if self._pending_topic_enrichment:
            changed_topics = set(self._pending_topic_enrichment)
            self._enrich_pending_relations(topic_type_list)
            self._refresh_bridge_qos(changed_topics)

        # ── 8. Retry action type resolution for any unresolved actions ────────
        for a in current_actions:
            if a not in self._action_type_cache:
                self._fetch_action_types(a, topic_type_map)

        # ── 9. Nav2 BT publisher liveness check ──────────────────────────────
        if self._nav2_bt_monitor_initialized and not self._nav2_bt_publisher_active:
            # Nothing publishes the followed log: a Nav2 that came up under
            # another namespace (or later than this agent) is followed instead.
            if f'{self._resolve_nav2_namespace()}/behavior_tree_log' != self._nav2_bt_log_topic:
                self._teardown_nav2_bt_monitor()
                self._init_nav2_bt_monitor()
        if self._nav2_bt_monitor_initialized:
            bt_rel = self._topic_relations.get(self._nav2_bt_log_topic, {})
            bt_pubs = bt_rel.get('publishers', set()) & current_nodes
            if self._nav2_bt_publisher_active and not bt_pubs:
                self._on_nav2_bt_gone()
            elif self._nav2_bt_publisher_active and bt_pubs and self._nav2_bt_tree_id is None:
                if self._load_and_parse_bt_xml():
                    self._on_bt_event({
                        'type': 'bt_tree', 'timestamp': time.time(),
                        'tree_id': self._nav2_bt_tree_id,
                        'tree': self._nav2_bt_tree_structure,
                        'nodes': [{**nd, 'status': 'IDLE'} for nd in self._nav2_bt_nodes_list],
                    })
            elif not self._nav2_bt_publisher_active and bt_pubs:
                self._nav2_bt_publisher_active = True
                if self._load_and_parse_bt_xml():
                    self._on_bt_event({
                        'type': 'bt_tree', 'timestamp': time.time(),
                        'tree_id': self._nav2_bt_tree_id,
                        'tree': self._nav2_bt_tree_structure,
                        'nodes': [{**nd, 'status': 'IDLE'} for nd in self._nav2_bt_nodes_list],
                    })

        # ── 10. Flush graph snapshots if anything changed ─────────────────────
        self._flush_graph_snapshots()

        # ── 11. Poll collectors ───────────────────────────────────────────────
        if self._ros2_control is not None:
            self._ros2_control.poll()
        if self._tf_tree is not None:
            self._tf_tree.follow_published_tf()
            self._tf_tree.poll()

    def _only_own_status_subscription(self, status_topic: str) -> bool:
        """True when an action's status topic exists only because this agent
        subscribes to it: no server publishes it and no other node reads it.
        Such a topic is not an action. Counting it as one would also keep it
        alive, since goal tracking subscribes to every action's status. An
        action with clients but no server still counts."""
        if self.count_publishers(status_topic) > 0:
            return False
        own = self.get_fully_qualified_name()
        return all(
            self._node_full_name(s.node_name, s.node_namespace) == own
            for s in self.get_subscriptions_info_by_topic(status_topic)
        )

    def _cancel_param_fetch_timer(self):
        """Cancel the one-shot delayed param-fetch timer after it fires."""
        t = self._param_fetch_timer
        if t is not None:
            t.cancel()
            self._param_fetch_timer = None

    # ──────────────────────────────────────────────
    # Initial full enrichment (called once on first tick)
    # ──────────────────────────────────────────────

    def _do_full_initial_enrichment(self, topic_type_list, node_pairs):
        topic_type_map = dict(topic_type_list)
        self._pending_topic_enrichment.clear()
        for topic in self._active_topics:
            try:
                pub_infos = self.get_publishers_info_by_topic(topic)
                sub_infos = self.get_subscriptions_info_by_topic(topic)
            except Exception:
                continue
            publishers  = {self._node_full_name(p.node_name, p.node_namespace) for p in pub_infos}
            subscribers = {self._node_full_name(s.node_name, s.node_namespace) for s in sub_infos}
            self._topic_relations[topic] = {
                'publishers':       publishers,
                'subscribers':      subscribers,
                'publisher_infos':  pub_infos,
                'subscriber_infos': sub_infos,
                'type': topic_type_map.get(topic, ['unknown'])[0],
            }
            self._topic_counts[topic] = (len(pub_infos), len(sub_infos))

    # ──────────────────────────────────────────────
    # Tier-2: batched relation enrichment
    # ──────────────────────────────────────────────

    def _enrich_pending_relations(self, topic_type_list=None):
        if not self._pending_topic_enrichment:
            return

        batch = set(self._pending_topic_enrichment)
        self._pending_topic_enrichment.clear()
        _t0 = time.time()
        self.get_logger().info(f"[enrich] {len(batch)} topics")

        if topic_type_list is not None:
            topic_type_map = dict(topic_type_list)
        else:
            topic_type_map = dict(self.get_topic_names_and_types())

        for topic in batch:
            if topic not in self._active_topics:
                continue
            try:
                pub_infos = self.get_publishers_info_by_topic(topic)
                sub_infos = self.get_subscriptions_info_by_topic(topic)
            except Exception as e:
                self.get_logger().debug(f"Enrichment failed for {topic}: {e}")
                continue

            publishers  = {self._node_full_name(p.node_name, p.node_namespace) for p in pub_infos}
            subscribers = {self._node_full_name(s.node_name, s.node_namespace) for s in sub_infos}
            old = self._topic_relations.get(topic)
            new_rel = {
                'publishers':       publishers,
                'subscribers':      subscribers,
                'publisher_infos':  pub_infos,
                'subscriber_infos': sub_infos,
                'type': topic_type_map.get(topic, ['unknown'])[0],
            }
            self._topic_relations[topic] = new_rel
            self._topic_counts[topic] = (len(pub_infos), len(sub_infos))

            if old is not None:
                if (subscribers != old['subscribers'] or publishers != old['publishers']
                        or len(pub_infos) != len(old['publisher_infos'])
                        or len(sub_infos) != len(old['subscriber_infos'])):
                    self._graph_dirty = True

                # Nav2 BT: publisher appeared/vanished on the followed log
                if topic == self._nav2_bt_log_topic and self._nav2_bt_monitor_initialized:
                    old_pubs = old['publishers']
                    if publishers and not old_pubs:
                        self._nav2_bt_publisher_active = True
                        if self._load_and_parse_bt_xml():
                            self._on_bt_event({
                                'type': 'bt_tree', 'timestamp': time.time(),
                                'tree_id': self._nav2_bt_tree_id,
                                'tree': self._nav2_bt_tree_structure,
                                'nodes': [{**nd, 'status': 'IDLE'} for nd in self._nav2_bt_nodes_list],
                            })
                    elif old_pubs and not publishers:
                        self._on_nav2_bt_gone()

        self.get_logger().info(f"[enrich] done in {(time.time()-_t0)*1000:.1f}ms")

    # ──────────────────────────────────────────────
    # Graph snapshot builders
    # ──────────────────────────────────────────────

    def _get_nodes_with_relations(self) -> dict:
        result = {}
        for fqn in self._active_nodes:
            result[fqn] = {
                'publishes':  [],
                'subscribes': [],
                'actions':    [],
                'services':   [],
                'parameters': self._node_parameter_cache.get(fqn, {}),
                'lifecycle_state': self._lifecycle_state_cache.get(fqn, None),
            }

        for topic, rel in self._topic_relations.items():
            pub_infos = rel.get('publisher_infos', [])
            sub_infos = rel.get('subscriber_infos', [])
            for p in pub_infos:
                fqn = self._node_full_name(p.node_name, p.node_namespace)
                if fqn in result:
                    result[fqn]['publishes'].append({
                        'topic': topic,
                        'qos': self._qos_to_dict(p.qos_profile),
                    })
            for s in sub_infos:
                fqn = self._node_full_name(s.node_name, s.node_namespace)
                if fqn in result:
                    result[fqn]['subscribes'].append({
                        'topic': topic,
                        'qos': self._qos_to_dict(s.qos_profile),
                    })

        for topic, rel in self._topic_relations.items():
            if topic.endswith('/_action/status') and rel['publishers']:
                action = topic.replace('/_action/status', '')
                for p in rel['publisher_infos']:
                    fqn = self._node_full_name(p.node_name, p.node_namespace)
                    if fqn in result and action not in result[fqn]['actions']:
                        result[fqn]['actions'].append(action)

        return result

    def _get_topics_with_relations(self) -> dict:
        result = {}
        for topic in self._active_topics:
            rel = self._topic_relations.get(topic, {})
            result[topic] = {
                'type': rel.get('type', 'unknown'),
                'publishers': [
                    {
                        'node': self._node_full_name(p.node_name, p.node_namespace),
                        'qos': self._qos_to_dict(p.qos_profile),
                    }
                    for p in rel.get('publisher_infos', [])
                ],
                'subscribers': [
                    {
                        'node': self._node_full_name(s.node_name, s.node_namespace),
                        'qos': self._qos_to_dict(s.qos_profile),
                    }
                    for s in rel.get('subscriber_infos', [])
                ],
            }
        return result

    def _get_actions_with_relations(self) -> dict:
        result = {}
        for action in self._active_actions:
            rel = self._topic_relations.get(f'{action}/_action/status', {})
            providers = [
                self._node_full_name(p.node_name, p.node_namespace)
                for p in rel.get('publisher_infos', [])
            ]
            result[action] = {
                'providers': providers,
                **(self._action_type_cache.get(action) or {}),
            }
        return result

    def _get_services_with_relations(self) -> dict:
        return {
            name: {'type': type_str, 'providers': []}
            for name, type_str in self._active_services.items()
        }

    def _get_graph_snapshot_locked(self) -> tuple[dict, dict, dict, dict]:
        """Gather all four graph relation dicts under _graph_check_lock.

        _check_graph_changes_locked() mutates _topic_relations / _active_*
        in place on the timer/executor thread; without this lock a caller
        on another thread (e.g. _send_initial_state on the websocket thread)
        can observe a torn dict mid-mutation (RuntimeError: dictionary
        changed size during iteration, or a silently incomplete snapshot).
        """
        with self._graph_check_lock:
            return (
                self._get_nodes_with_relations(),
                self._get_topics_with_relations(),
                self._get_actions_with_relations(),
                self._get_services_with_relations(),
            )

    # ──────────────────────────────────────────────
    # Delta-send: flush graph snapshots after each tick
    # ──────────────────────────────────────────────

    def _flush_graph_snapshots(self):
        if not self._graph_dirty or not self.ws or not self.loop:
            return

        # Only clear the dirty flag once every snapshot has actually been
        # built and enqueued. If anything below raises, the flag is put
        # back so the next trigger retries instead of the client silently
        # never receiving this update.
        try:
            nodes = self._get_nodes_with_relations()
            if nodes != self._last_sent_nodes:
                self.get_logger().info(f"[flush] nodes ({len(nodes)} nodes)")
                self._last_sent_nodes = nodes.copy()
                self._enqueue({'type': 'nodes', 'data': nodes, 'timestamp': time.time()})

            topics = self._get_topics_with_relations()
            if topics != self._last_sent_topics:
                self.get_logger().info(f"[flush] topics ({len(topics)} topics)")
                self._last_sent_topics = topics.copy()
                self._enqueue({'type': 'topics', 'data': topics, 'timestamp': time.time()})

            actions = self._get_actions_with_relations()
            if actions != self._last_sent_actions:
                self.get_logger().info(f"[flush] actions ({len(actions)} actions)")
                self._last_sent_actions = actions.copy()
                self._enqueue({'type': 'actions', 'data': actions, 'timestamp': time.time()})

            services = self._get_services_with_relations()
            if services != self._last_sent_services:
                self.get_logger().info(f"[flush] services ({len(services)} services)")
                self._last_sent_services = services.copy()
                self._enqueue({'type': 'services', 'data': services, 'timestamp': time.time()})
        except Exception:
            self._graph_dirty = True
            raise
        else:
            self._graph_dirty = False

    # ──────────────────────────────────────────────
    # C++ graph watcher integration
    # ──────────────────────────────────────────────

    def _do_startup_check(self):
        """One-shot timer: run the initial graph scan then cancel itself."""
        self._startup_check_timer.cancel()
        if self._watcher_proc is not None:
            rc = self._watcher_proc.poll()
            if rc is not None:
                self.get_logger().error(
                    f"[graph] graph_watcher exited unexpectedly (rc={rc}) — "
                    "no graph events will be received"
                )
            else:
                self.get_logger().info(
                    f"[graph] graph_watcher healthy (pid={self._watcher_proc.pid})"
                )
        if not self._first_graph_check_done:
            self._check_graph_changes()

    def _check_graph_watcher(self):
        """Starts the graph watcher again if it was running and has exited.

        Without it the bridge hears of no graph change, and the app keeps the
        robot's last graph until the bridge restarts. A robot with no watcher
        binary to start (None from the start) is left as it is.
        """
        proc = self._watcher_proc
        if proc is None or proc.poll() is None:
            return
        now = time.monotonic()
        restarts = [t for t in getattr(self, '_watcher_restarts', []) if now - t < WATCHER_RESTART_WINDOW_S]
        self._watcher_restarts = restarts
        if len(restarts) >= WATCHER_QUICK_RESTARTS and now - restarts[-1] < WATCHER_RESTART_WINDOW_S:
            return
        self.get_logger().error(
            f"[graph] graph_watcher exited (rc={proc.returncode}) — starting it again"
        )
        restarts.append(now)
        new_proc = start_graph_watcher()
        if new_proc is None:
            return
        self._watcher_proc = new_proc
        self._forward_watcher_stderr(new_proc)
        # Changes while it was down were never announced: look once now.
        self._trigger_graph_poll()

    def _note_graph(self, nodes: set, topics: set):
        """Records when the set of nodes or topics last changed."""
        if (nodes, topics) != self._graph_seen:
            self._graph_seen = (nodes, topics)
            self._graph_changed_at = time.monotonic()

    def _wait_for_graph_to_settle(self):
        """Blocks until the first graph scan has run and no node or topic has
        changed for GRAPH_SETTLE_S, and never past GRAPH_SETTLE_MAX_S after
        the agent started. A graph sent while discovery is still finding the
        robot's nodes is partial, and the gateway would record every node it
        lacks as removed, then added back. On a reconnect the graph settled
        long ago, and this returns at once."""
        while time.monotonic() - self._started_at < GRAPH_SETTLE_MAX_S:
            if (self._initial_scan_complete.is_set()
                    and not self._graph_check_lock.locked()
                    and time.monotonic() - self._graph_changed_at >= GRAPH_SETTLE_S):
                return
            time.sleep(0.1)

    def _on_graph_changed(self, _msg: EmptyMsg):
        """Debounced callback fired by the C++ osiris_graph_watcher node."""
        self.get_logger().debug("[graph] event received")
        self._trigger_graph_poll()

    def _trigger_graph_poll(self):
        """Single debounced entry point for all graph poll triggers.

        The first trigger schedules a poll graph_debounce_interval later, and
        triggers until then join it. A burst becomes one poll, and a steady
        stream of triggers (a node changing a parameter every few hundred ms)
        still gets one poll per interval instead of postponing it for good.
        """
        if self._graph_debounce_timer is not None:
            return
        _interval = self.get_parameter('graph_debounce_interval').get_parameter_value().double_value
        self._graph_debounce_timer = threading.Timer(_interval, self._debounce_fire)
        self._graph_debounce_timer.daemon = True
        self._graph_debounce_timer.start()

    def _debounce_fire(self):
        """Called from threading.Timer — run the graph poll directly."""
        # Cleared first, so a trigger during the poll schedules the next one.
        self._graph_debounce_timer = None
        self.get_logger().debug("[graph] watcher triggered poll")
        self._check_graph_changes()

    # ──────────────────────────────────────────────
    # Helpers
    # ──────────────────────────────────────────────

    @staticmethod
    def _node_full_name(name: str, namespace: str) -> str:
        ns = namespace if namespace.endswith('/') else namespace + '/'
        return ns + name

    @staticmethod
    def _qos_to_dict(qos) -> dict | None:
        if not qos:
            return None
        return {
            'reliability': qos.reliability.name if hasattr(qos.reliability, 'name') else str(qos.reliability),
            'durability':  qos.durability.name  if hasattr(qos.durability,  'name') else str(qos.durability),
            'history':     qos.history.name     if hasattr(qos.history,     'name') else str(qos.history),
            'depth':       qos.depth,
            'liveliness':  qos.liveliness.name  if hasattr(qos.liveliness,  'name') else str(qos.liveliness),
        }
