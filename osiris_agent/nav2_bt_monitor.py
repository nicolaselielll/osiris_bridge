"""Behavior tree state: Nav2 BT monitoring and the cached tree sent with initial_state."""

import hashlib
import os
import time
import xml.etree.ElementTree as ET

from .bt_status import resting

# Nav2's navigators that run a behavior tree: the bt_navigator parameter that
# names each one's tree, and the file Nav2 uses without it.
NAV2_NAVIGATORS = (
    ('navigate_to_pose', 'default_nav_to_pose_bt_xml',
     'navigate_to_pose_w_replanning_and_recovery.xml'),
    ('navigate_through_poses', 'default_nav_through_poses_bt_xml',
     'navigate_through_poses_w_replanning_and_recovery.xml'),
)
ACTIVE_GOAL_STATUSES = (1, 2, 3)  # accepted, executing, canceling
# Nav2 logs a change on every tick (up to 100 a second); they go out together
# at most this often, every one of them, in order.
NAV2_BT_FLUSH_S = 0.1
# A goal that ends without its last tick in the log shows its tree at rest
# after this long, unless the log moved on. Nav2 Jazzy publishes a tick's
# changes just before the next tick, so a goal's last tick reaches the log
# only when the navigator's next goal starts; a canceled goal never has one.
NAV2_BT_REST_AFTER_S = 1.0
# The navigator's result for a goal, as its tree's root ended.
GOAL_ROOT_OUTCOMES = {4: 'SUCCESS', 6: 'FAILURE'}  # succeeded, aborted


def parse_bt_xml(xml_content: str) -> dict | None:
    """The main tree of a behavior tree file, numbered as BT.CPP numbers its
    nodes when it builds the tree (depth first from 1, a subtree's nodes right
    after its SubTree node), so the uids in Nav2's log name the same nodes."""
    root = ET.fromstring(xml_content)
    trees = {bt.attrib.get('ID', ''): bt for bt in root.iter('BehaviorTree')}
    if not trees:
        return None
    main = trees.get(root.attrib.get('main_tree_to_execute', ''))
    if main is None:
        main = trees.get('MainTree')
    if main is None:
        main = next(iter(trees.values()))
    nodes_list: list = []
    counter = [0]

    def elem_to_node(elem, depth):
        counter[0] += 1
        uid = counter[0]
        name = elem.attrib.get('name', elem.attrib.get('ID', elem.tag))
        nodes_list.append({'uid': uid, 'name': name, 'tag': elem.tag})
        node = {'tag': elem.tag, 'name': name, 'uid': uid, 'attributes': dict(elem.attrib)}
        if elem.tag in ('SubTree', 'SubTreePlus'):
            sub = trees.get(elem.attrib.get('ID', ''))
            kids = [elem_to_node(list(sub)[0], depth + 1)] if sub is not None and len(sub) and depth < 32 else []
        else:
            kids = [elem_to_node(c, depth + 1) for c in elem]
        if kids:
            node['children'] = kids
        return node

    if not len(main):
        return None
    structure = elem_to_node(list(main)[0], 0)
    return {
        'tree_id': hashlib.sha1(xml_content.encode()).hexdigest()[:16],
        'structure': structure,
        'nodes': nodes_list,
    }


class Nav2Tree:
    """One navigator's behavior tree and the status of each of its nodes."""

    def __init__(self, navigator: str, path: str, parsed: dict):
        self.navigator = navigator
        self.path = path
        self.tree_id = parsed['tree_id']
        # The pane heads the tree with its navigator: NavigateToPose.
        self.structure = {**parsed['structure'], 'tree_name': ''.join(w.capitalize() for w in navigator.split('_'))}
        self.nodes = parsed['nodes']
        self.root_uid = self.nodes[0]['uid'] if self.nodes else None
        self.by_uid = {n['uid']: n for n in self.nodes}
        self.by_name: dict[str, list[int]] = {}
        for n in self.nodes:
            self.by_name.setdefault(n['name'], []).append(n['uid'])
        self.parent: dict[int, int] = {}
        stack = [parsed['structure']]
        while stack:
            node = stack.pop()
            for child in node.get('children', []):
                self.parent[child['uid']] = node['uid']
                stack.append(child)
        self.statuses: dict[int, str] = {}   # as the pane shows them
        self.logged: dict[int, str] = {}     # as the log last gave them

    def fits(self, events) -> bool:
        """Whether every change in a log message names a node of this tree: by
        uid and name where the log has uids (Jazzy on), by name before."""
        for e in events:
            uid = getattr(e, 'uid', 0)
            if uid:
                node = self.by_uid.get(uid)
                if node is None or node['name'] != e.node_name:
                    return False
            elif e.node_name not in self.by_name:
                return False
        return True

    def uid_for(self, event):
        uid = getattr(event, 'uid', 0)
        node = self.by_uid.get(uid) if uid else None
        if node is not None and node['name'] == event.node_name:
            return uid
        # A log without uids names nodes, and several can share a name (Nav2's
        # own tree has two ComputePathToPose and three Sequence). The change is
        # the one whose node was in the change's previous status, under a
        # parent that is running (a node starts, ends and resets only while
        # its parent runs): the first such in tree order when a node starts
        # (parents start before their children), the last otherwise (children
        # finish and reset first).
        candidates = self.by_name.get(event.node_name)
        if not candidates:
            return None
        if len(candidates) == 1:
            return candidates[0]
        matching = [u for u in candidates if self.logged.get(u, 'IDLE') == event.previous_status] or candidates
        matching = [u for u in matching if self.parent.get(u) is None or self.logged.get(self.parent[u]) == 'RUNNING'] or matching
        return matching[0] if event.previous_status == 'IDLE' else matching[-1]

    def apply(self, uid: int, status: str, previous: str):
        self.logged[uid] = status
        self.statuses[uid] = resting(previous) if status == 'IDLE' else status

    def rest(self, root_outcome: str | None = None) -> bool:
        """The tree halted: every node idle, remembering how it last ended. A
        root still running in the log ended as its goal did (root_outcome)."""
        changed = False
        if root_outcome and self.statuses.get(self.root_uid) == 'RUNNING':
            self.statuses[self.root_uid] = root_outcome
        for uid, status in list(self.statuses.items()):
            if resting(status) != status:
                self.statuses[uid] = resting(status)
                changed = True
        return changed

    def tree_event(self) -> dict:
        statuses = dict(self.statuses)
        return {
            'type': 'bt_tree', 'timestamp': time.time(),
            'tree_id': self.tree_id,
            'tree': self.structure,
            'nodes': [{**n, 'status': statuses.get(n['uid'], 'IDLE')} for n in self.nodes],
        }


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
            self._nav2_bt_publisher_active = False
            self._nav2_bt_trees: dict[str, Nav2Tree] = {}
            self._nav2_bt_active: Nav2Tree | None = None   # the tree the pane shows
            self._nav2_bt_pending: list = []               # changes not sent yet
            self._nav2_bt_goal_active: dict[str, bool] = {}
            self._nav2_bt_last_log = 0.0
            self._nav2_bt_rest_timer = None
            self._nav2_bt_rest_due: tuple | None = None    # (tree, goal ended at)
            self._nav2_bt_unmatched_logged = False
            self._nav2_bt_unreadable: set[str] = set()
            self._nav2_bt_default_paths: dict[str, str] = {}
            self._nav2_bt_log_sub = self.create_subscription(
                BehaviorTreeLog, self._nav2_bt_log_topic, self._on_nav2_bt_log, 100
            )
            self._nav2_bt_goal_status_subs = [
                self.create_subscription(
                    GoalStatusArray, f'{ns}/{navigator}/_action/status',
                    lambda msg, navigator=navigator: self._on_nav2_goal_status(navigator, msg), 10,
                )
                for navigator, _, _ in NAV2_NAVIGATORS
            ]
            # Runs only while changes wait, so an idle Nav2 costs no wake-ups.
            self._nav2_bt_flush_timer = self.create_timer(NAV2_BT_FLUSH_S, self._flush_nav2_bt_changes)
            self._nav2_bt_flush_timer.cancel()
            self._nav2_bt_monitor_initialized = True
            # If bt_navigator is already publishing, load the trees so the
            # startup bt_state event includes one.
            if self.count_publishers(self._nav2_bt_log_topic) > 0:
                self._on_nav2_bt_publisher()
        except Exception as e:
            self.get_logger().debug(f"Nav2 BT monitoring unavailable: {e}")

    def _teardown_nav2_bt_monitor(self):
        if not self._nav2_bt_monitor_initialized:
            return
        self._on_nav2_bt_gone()  # notifies client the tree is gone, resets tree-state fields
        self.destroy_subscription(self._nav2_bt_log_sub)
        for sub in self._nav2_bt_goal_status_subs:
            self.destroy_subscription(sub)
        self.destroy_timer(self._nav2_bt_flush_timer)
        self._nav2_bt_log_sub = None
        self._nav2_bt_goal_status_subs = []
        self._nav2_bt_flush_timer = None
        self._nav2_bt_monitor_initialized = False
        self.get_logger().info('Nav2 BT monitoring stopped')

    def _nav2_bt_xml_path(self, param: str, default_file: str) -> str:
        path = self._node_parameter_cache.get(self._nav2_bt_navigator, {}).get(param, '')
        if path:
            return path
        if default_file not in self._nav2_bt_default_paths:
            try:
                from ament_index_python.packages import get_package_share_directory
                share = get_package_share_directory('nav2_bt_navigator')
                self._nav2_bt_default_paths[default_file] = os.path.join(share, 'behavior_trees', default_file)
            except Exception:
                self._nav2_bt_default_paths[default_file] = ''
        return self._nav2_bt_default_paths[default_file]

    def _load_nav2_trees(self):
        """Each navigator's tree, read again when bt_navigator's parameters (once
        fetched) name another file than the one read."""
        for navigator, param, default_file in NAV2_NAVIGATORS:
            path = self._nav2_bt_xml_path(param, default_file)
            current = self._nav2_bt_trees.get(navigator)
            if not path or (current is not None and current.path == path) or path in self._nav2_bt_unreadable:
                continue
            try:
                with open(path) as f:
                    parsed = parse_bt_xml(f.read())
            except Exception as e:
                self._nav2_bt_unreadable.add(path)
                self.get_logger().error(f"Cannot read BT XML '{path}': {e}")
                continue
            if not parsed:
                self._nav2_bt_unreadable.add(path)
                continue
            tree = Nav2Tree(navigator, path, parsed)
            self._nav2_bt_trees[navigator] = tree
            if self._nav2_bt_active is current and current is not None:
                self._flush_nav2_bt_changes()
                self._nav2_bt_active = tree
                self._on_bt_event(tree.tree_event())

    def _on_nav2_bt_publisher(self):
        """bt_navigator publishes its log: the pane gets navigate_to_pose's tree
        until another navigator's runs."""
        self._nav2_bt_publisher_active = True
        self._load_nav2_trees()
        if self._nav2_bt_active is None:
            tree = self._nav2_bt_trees.get(NAV2_NAVIGATORS[0][0]) or next(iter(self._nav2_bt_trees.values()), None)
            if tree is not None:
                self._nav2_bt_active = tree
                self._on_bt_event(tree.tree_event())

    def _nav2_tree_for(self, events):
        """The tree a log message comes from. Both navigators log to the same
        topic and number their nodes from 1, so the message's own changes tell
        them apart; a navigator with a goal running decides a tie."""
        fitting = [t for t in self._nav2_bt_trees.values() if t.fits(events)]
        if len(fitting) > 1:
            running = [t for t in fitting if self._nav2_bt_goal_active.get(t.navigator)]
            fitting = running if len(running) == 1 else (
                [self._nav2_bt_active] if self._nav2_bt_active in fitting else fitting)
        if fitting:
            return fitting[0]
        if not self._nav2_bt_unmatched_logged:
            self._nav2_bt_unmatched_logged = True
            names = sorted({e.node_name for e in events})[:5]
            self.get_logger().warning(
                f"[bt] the behavior tree log names nodes that aren't in the trees read "
                f"({', '.join(t.path for t in self._nav2_bt_trees.values())}): {names}"
            )
        return None

    def _on_nav2_bt_log(self, msg):
        events = list(msg.event_log)
        if not events:
            return
        self._nav2_bt_publisher_active = True
        self._load_nav2_trees()
        tree = self._nav2_tree_for(events)
        if tree is None:
            return
        self._nav2_bt_last_log = time.time()
        if tree is not self._nav2_bt_active:
            self._flush_nav2_bt_changes()
            self._nav2_bt_active = tree
            self._on_bt_event(tree.tree_event())
        for e in events:
            uid = tree.uid_for(e)
            if uid is None:
                continue
            node = tree.by_uid[uid]
            tree.apply(uid, e.current_status, e.previous_status)
            if not self._nav2_bt_pending:
                self._nav2_bt_flush_timer.reset()
            self._nav2_bt_pending.append({
                'uid': uid, 'name': node['name'], 'tag': node['tag'],
                'status': e.current_status, 'previous_status': e.previous_status,
            })
        if tree.statuses.get(tree.root_uid) in ('SUCCESS', 'FAILURE'):
            # The root ended (a message can also hold the end of the last goal
            # and the start of the next): Nav2 halts the tree right after, and
            # that reaches the log only when the navigator's next goal starts.
            self._flush_nav2_bt_changes()
            if tree.rest():
                self._on_bt_event(tree.tree_event())

    def _flush_nav2_bt_changes(self):
        if self._nav2_bt_flush_timer is not None:
            self._nav2_bt_flush_timer.cancel()
        if not self._nav2_bt_pending or self._nav2_bt_active is None:
            return
        changes, self._nav2_bt_pending = self._nav2_bt_pending, []
        self._on_bt_event({
            'type': 'bt_status', 'timestamp': time.time(),
            'tree_id': self._nav2_bt_active.tree_id,
            'changes': changes,
        })

    def _on_nav2_goal_status(self, navigator: str, msg):
        active = any(s.status in ACTIVE_GOAL_STATUSES for s in msg.status_list)
        was_active = self._nav2_bt_goal_active.get(navigator, False)
        self._nav2_bt_goal_active[navigator] = active
        tree = self._nav2_bt_trees.get(navigator)
        if was_active and not active and tree is not None:
            outcome = GOAL_ROOT_OUTCOMES.get(msg.status_list[-1].status) if msg.status_list else None
            self._nav2_bt_rest_due = (tree, time.time(), outcome)
            if self._nav2_bt_rest_timer is None:
                self._nav2_bt_rest_timer = self.create_timer(NAV2_BT_REST_AFTER_S, self._on_nav2_bt_rest_timer)

    def _on_nav2_bt_rest_timer(self):
        self.destroy_timer(self._nav2_bt_rest_timer)
        self._nav2_bt_rest_timer = None
        due, self._nav2_bt_rest_due = self._nav2_bt_rest_due, None
        if due is None:
            return
        tree, ended_at, outcome = due
        if self._nav2_bt_goal_active.get(tree.navigator) or self._nav2_bt_last_log >= ended_at:
            return
        self._flush_nav2_bt_changes()
        if tree.rest(outcome) and tree is self._nav2_bt_active:
            self._on_bt_event(tree.tree_event())

    def _on_nav2_bt_gone(self):
        if not self._nav2_bt_trees and not self._nav2_bt_publisher_active:
            return
        self.get_logger().info("Nav2 BT gone — clearing BT state")
        self._nav2_bt_publisher_active = False
        self._nav2_bt_trees = {}
        self._nav2_bt_active = None
        self._nav2_bt_pending = []
        self._nav2_bt_goal_active = {}
        self._nav2_bt_rest_due = None
        self._on_bt_event({
            'type': 'bt_tree', 'timestamp': time.time(),
            'tree_id': None, 'tree': None, 'nodes': [],
        })

    def _build_startup_bt_state_event(self) -> dict:
        tree = self._nav2_bt_active if self._nav2_bt_monitor_initialized else None
        collector = getattr(self, '_bt_collector', None)
        src = tree.tree_event() if tree is not None else (
            collector.tree_event() if collector is not None else None) or self._cached_bt_tree_event
        if src:
            return {
                'type': 'bt_state', 'timestamp': src.get('timestamp', time.time()),
                'tree_id': src.get('tree_id'), 'tree': src.get('tree'),
                'nodes': src.get('nodes', []),
                'blackboard': src.get('blackboard'),
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
