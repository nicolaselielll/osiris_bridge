"""Behavior tree state: Nav2 BT monitoring and the cached tree sent with initial_state."""

import os
import time


class Nav2BtMonitorMixin:

    def _on_bt_event(self, event: dict):
        if event.get('type') == 'bt_tree':
            self._cached_bt_tree_event = event if event.get('tree_id') else None
        self._enqueue(event)

    def _resolve_nav2_namespace(self) -> str:
        """Namespace of the Nav2 whose behavior tree to follow ('' for the root):
        this agent's own namespace if a behavior tree log is published there,
        else the only one published, else the root. Only published logs count:
        this agent's own subscription makes a log topic exist without a publisher."""
        suffix = '/behavior_tree_log'
        candidates = sorted(
            t[:-len(suffix)] for t, _ in self.get_topic_names_and_types()
            if t.endswith(suffix) and self.count_publishers(t) > 0
        )
        own = self.get_namespace().rstrip('/')
        if own in candidates:
            return own
        if len(candidates) == 1:
            return candidates[0]
        return own if not candidates else ''

    def _init_nav2_bt_monitor(self):
        try:
            from nav2_msgs.msg import BehaviorTreeLog
            from action_msgs.msg import GoalStatusArray
            ns = self._resolve_nav2_namespace()
            self._nav2_bt_log_topic = f'{ns}/behavior_tree_log'
            self._nav2_bt_navigator = f'{ns}/bt_navigator'
            self._nav2_bt_statuses:           dict[str, str] = {}
            self._nav2_bt_last_sent_statuses:  dict[str, str] = {}  # what client currently has
            self._nav2_bt_session_active   = False
            self._nav2_bt_publisher_active = False
            self._nav2_bt_tree_id          = None
            self._nav2_bt_tree_structure   = None
            self._nav2_bt_nodes_list:      list = []
            self._nav2_bt_name_to_uid:     dict = {}
            self._nav2_bt_log_sub = self.create_subscription(
                BehaviorTreeLog, self._nav2_bt_log_topic, self._on_nav2_bt_log, 10
            )
            self._nav2_bt_goal_status_sub = self.create_subscription(
                GoalStatusArray,
                f'{ns}/navigate_to_pose/_action/status',
                self._on_nav2_goal_status,
                10,
            )
            self._nav2_bt_monitor_initialized = True
            # If bt_navigator is already publishing, pre-parse the XML so
            # the startup bt_state event includes the tree structure.
            if self.count_publishers(self._nav2_bt_log_topic) > 0:
                self._nav2_bt_publisher_active = True
                self._load_and_parse_bt_xml()
        except Exception as e:
            self.get_logger().debug(f"Nav2 BT monitoring unavailable: {e}")

    def _teardown_nav2_bt_monitor(self):
        if not self._nav2_bt_monitor_initialized:
            return
        self._on_nav2_bt_gone()  # notifies client the tree is gone, resets tree-state fields
        self.destroy_subscription(self._nav2_bt_log_sub)
        self.destroy_subscription(self._nav2_bt_goal_status_sub)
        self._nav2_bt_log_sub = None
        self._nav2_bt_goal_status_sub = None
        self._nav2_bt_monitor_initialized = False
        self.get_logger().info('Nav2 BT monitoring stopped')

    def _load_and_parse_bt_xml(self) -> bool:
        if self._nav2_bt_tree_id is not None:
            return True
        import hashlib
        import xml.etree.ElementTree as ET

        xml_path = self._node_parameter_cache.get(self._nav2_bt_navigator, {}).get(
            'default_nav_to_pose_bt_xml', ''
        )
        if not xml_path:
            try:
                from ament_index_python.packages import get_package_share_directory
                nav2_share = get_package_share_directory('nav2_bt_navigator')
                xml_path = os.path.join(
                    nav2_share, 'behavior_trees',
                    'navigate_to_pose_w_replanning_and_recovery.xml',
                )
            except Exception:
                return False

        try:
            with open(xml_path) as f:
                xml_content = f.read()
        except Exception as e:
            self.get_logger().error(f"Cannot read BT XML '{xml_path}': {e}")
            return False

        try:
            root_elem = ET.fromstring(xml_content)
            bt_elem = root_elem.find('.//BehaviorTree')
            if bt_elem is None:
                return False

            nodes_list: list = []
            name_to_uid: dict = {}
            uid_counter = [1]

            def elem_to_node(elem):
                name = elem.attrib.get('name', elem.attrib.get('ID', elem.tag))
                uid = uid_counter[0]; uid_counter[0] += 1
                name_to_uid[name] = uid
                nodes_list.append({'uid': uid, 'name': name, 'tag': elem.tag})
                node = {
                    'tag': elem.tag, 'name': name, 'uid': uid,
                    'attributes': dict(elem.attrib),
                }
                kids = [elem_to_node(c) for c in elem]
                if kids:
                    node['children'] = kids
                return node

            bt_children = list(bt_elem)
            tree_structure = elem_to_node(bt_children[0]) if bt_children else {}
            self._nav2_bt_tree_structure = tree_structure
            self._nav2_bt_nodes_list     = nodes_list
            self._nav2_bt_name_to_uid    = name_to_uid
            self._nav2_bt_tree_id = hashlib.sha1(xml_content.encode()).hexdigest()[:16]
            return True
        except Exception as e:
            self.get_logger().error(f"Failed to parse BT XML: {e}")
            return False

    def _on_nav2_bt_log(self, msg):
        if not self._nav2_bt_publisher_active:
            return
        if not self._load_and_parse_bt_xml():
            return

        # Collapse all transitions in this log tick to the final status per node.
        # The event_log can contain multiple entries for the same node (e.g.
        # RUNNING → FAILURE → IDLE) — only the last one matters to the client.
        has_running = False
        final_per_node: dict[str, str] = {}  # node_name → final status this tick
        for change in msg.event_log:
            self._nav2_bt_statuses[change.node_name] = change.current_status
            final_per_node[change.node_name] = change.current_status
            if change.current_status == 'RUNNING':
                has_running = True

        if has_running and not self._nav2_bt_session_active:
            self.get_logger().info("[bt] navigation session started")
            self._nav2_bt_session_active = True
            # Full tree send — sync last-sent cache
            self._nav2_bt_last_sent_statuses = {
                nd['name']: self._nav2_bt_statuses.get(nd['name'], 'IDLE')
                for nd in self._nav2_bt_nodes_list
            }
            self._on_bt_event({
                'type': 'bt_tree', 'timestamp': time.time(),
                'tree_id': self._nav2_bt_tree_id,
                'tree': self._nav2_bt_tree_structure,
                'nodes': [
                    {**nd, 'status': self._nav2_bt_last_sent_statuses.get(nd['name'], 'IDLE')}
                    for nd in self._nav2_bt_nodes_list
                ],
            })
            return  # full tree already sent; skip bt_status this tick

        # Only send nodes whose final status this tick differs from what client has
        changes = []
        for node_name, status in final_per_node.items():
            if self._nav2_bt_last_sent_statuses.get(node_name) == status:
                continue
            uid = self._nav2_bt_name_to_uid.get(node_name)
            if uid is not None:
                changes.append({'uid': uid, 'name': node_name, 'tag': '', 'status': status})
                self._nav2_bt_last_sent_statuses[node_name] = status

        if changes:
            self._on_bt_event({
                'type': 'bt_status', 'timestamp': time.time(),
                'tree_id': self._nav2_bt_tree_id,
                'changes': changes,
            })

    def _on_nav2_goal_status(self, msg):
        has_active = any(s.status in (1, 2, 3) for s in msg.status_list)
        if self._nav2_bt_session_active and not has_active:
            self.get_logger().info("[bt] navigation session ended")
            self._nav2_bt_session_active = False
            self._nav2_bt_statuses.clear()
            if self._nav2_bt_tree_id:
                self._on_bt_event({
                    'type': 'bt_tree', 'timestamp': time.time(),
                    'tree_id': self._nav2_bt_tree_id,
                    'tree': self._nav2_bt_tree_structure,
                    'nodes': [{**nd, 'status': 'IDLE'} for nd in self._nav2_bt_nodes_list],
                })

    def _on_nav2_bt_gone(self):
        if self._nav2_bt_tree_id is None and not self._nav2_bt_publisher_active:
            return
        self.get_logger().info("Nav2 BT gone — clearing BT state")
        self._nav2_bt_publisher_active = False
        self._nav2_bt_session_active   = False
        self._nav2_bt_statuses.clear()
        self._nav2_bt_last_sent_statuses.clear()
        self._nav2_bt_tree_id          = None
        self._nav2_bt_tree_structure   = None
        self._nav2_bt_nodes_list       = []
        self._nav2_bt_name_to_uid      = {}
        self._on_bt_event({
            'type': 'bt_tree', 'timestamp': time.time(),
            'tree_id': None, 'tree': None, 'nodes': [],
        })

    def _build_startup_bt_state_event(self) -> dict:
        src = self._cached_bt_tree_event
        if src:
            return {
                'type': 'bt_state', 'timestamp': src.get('timestamp', time.time()),
                'tree_id': src.get('tree_id'), 'tree': src.get('tree'),
                'nodes': src.get('nodes', []),
                'blackboard': src.get('blackboard'),
            }
        if (
            self._nav2_bt_monitor_initialized
            and self._nav2_bt_tree_id
            and self._nav2_bt_tree_structure
        ):
            return {
                'type': 'bt_state', 'timestamp': time.time(),
                'tree_id': self._nav2_bt_tree_id,
                'tree': self._nav2_bt_tree_structure,
                'nodes': [
                    {**nd, 'status': self._nav2_bt_statuses.get(nd['name'], 'IDLE')}
                    for nd in self._nav2_bt_nodes_list
                ],
                'blackboard': None,
            }
        return {
            'type': 'bt_state', 'timestamp': time.time(),
            'tree_id': None, 'tree': None, 'nodes': [], 'blackboard': None,
        }

    def _bt_snapshot_from_state_event(self, src: dict) -> dict:
        return {
            'timestamp': src.get('timestamp', time.time()),
            'tree_id': src.get('tree_id'),
            'tree': src.get('tree'),
            'nodes': src.get('nodes', []),
            'blackboard': src.get('blackboard'),
        }
