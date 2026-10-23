"""Nav2's behavior tree reaches the pane change by change, in the log's order,
on the node the log names, from the navigator whose tree logged it."""

import types

from osiris_agent.nav2_bt_monitor import Nav2BtMonitorMixin, parse_bt_xml

# Nav2 Jazzy's navigate_to_pose_w_replanning_and_recovery.xml: two nodes named
# ComputePathToPose, two FollowPath and three Sequence.
NAV_TO_POSE = """<root BTCPP_format="4" main_tree_to_execute="MainTree">
  <BehaviorTree ID="MainTree">
    <RecoveryNode number_of_retries="6" name="NavigateRecovery">
      <PipelineSequence name="NavigateWithReplanning">
        <ControllerSelector selected_controller="{selected_controller}" default_controller="FollowPath" topic_name="controller_selector"/>
        <PlannerSelector selected_planner="{selected_planner}" default_planner="GridBased" topic_name="planner_selector"/>
        <RateController hz="1.0">
          <RecoveryNode number_of_retries="1" name="ComputePathToPose">
            <ComputePathToPose goal="{goal}" path="{path}" planner_id="{selected_planner}" error_code_id="{compute_path_error_code}"/>
            <Sequence>
              <WouldAPlannerRecoveryHelp error_code="{compute_path_error_code}"/>
              <ClearEntireCostmap name="ClearGlobalCostmap-Context" service_name="global_costmap/clear_entirely_global_costmap"/>
            </Sequence>
          </RecoveryNode>
        </RateController>
        <RecoveryNode number_of_retries="1" name="FollowPath">
          <FollowPath path="{path}" controller_id="{selected_controller}" error_code_id="{follow_path_error_code}"/>
          <Sequence>
            <WouldAControllerRecoveryHelp error_code="{follow_path_error_code}"/>
            <ClearEntireCostmap name="ClearLocalCostmap-Context" service_name="local_costmap/clear_entirely_local_costmap"/>
          </Sequence>
        </RecoveryNode>
      </PipelineSequence>
      <Sequence>
        <Fallback>
          <WouldAControllerRecoveryHelp error_code="{follow_path_error_code}"/>
          <WouldAPlannerRecoveryHelp error_code="{compute_path_error_code}"/>
        </Fallback>
        <ReactiveFallback name="RecoveryFallback">
          <GoalUpdated/>
          <RoundRobin name="RecoveryActions">
            <Sequence name="ClearingActions">
              <ClearEntireCostmap name="ClearLocalCostmap-Subtree" service_name="local_costmap/clear_entirely_local_costmap"/>
              <ClearEntireCostmap name="ClearGlobalCostmap-Subtree" service_name="global_costmap/clear_entirely_global_costmap"/>
            </Sequence>
            <Spin spin_dist="1.57" error_code_id="{spin_error_code}"/>
            <Wait wait_duration="5.0"/>
            <BackUp backup_dist="0.30" backup_speed="0.15" error_code_id="{backup_code_id}"/>
          </RoundRobin>
        </ReactiveFallback>
      </Sequence>
    </RecoveryNode>
  </BehaviorTree>
</root>"""

THROUGH_POSES = """<root BTCPP_format="4" main_tree_to_execute="MainTree">
  <BehaviorTree ID="MainTree">
    <RecoveryNode number_of_retries="6" name="NavigateRecovery">
      <PipelineSequence name="NavigateWithReplanning">
        <ControllerSelector selected_controller="{c}" default_controller="FollowPath" topic_name="controller_selector"/>
        <PlannerSelector selected_planner="{p}" default_planner="GridBased" topic_name="planner_selector"/>
        <RateController hz="0.333">
          <RecoveryNode number_of_retries="1" name="ComputePathThroughPoses">
            <ComputePathThroughPoses goals="{goals}" path="{path}" planner_id="{p}"/>
            <ClearEntireCostmap name="ClearGlobalCostmap-Context" service_name="global_costmap/clear_entirely_global_costmap"/>
          </RecoveryNode>
        </RateController>
        <FollowPath path="{path}" controller_id="{c}"/>
      </PipelineSequence>
      <Wait wait_duration="5.0"/>
    </RecoveryNode>
  </BehaviorTree>
</root>"""

WITH_SUBTREE = """<root BTCPP_format="4" main_tree_to_execute="Main">
  <BehaviorTree ID="Main">
    <Sequence name="Root">
      <SubTree ID="Part" name="First"/>
      <Wait name="Pause" wait_duration="1.0"/>
      <SubTree ID="Part" name="Second"/>
    </Sequence>
  </BehaviorTree>
  <BehaviorTree ID="Part">
    <Sequence name="PartSeq">
      <Spin name="Turn"/>
    </Sequence>
  </BehaviorTree>
</root>"""


def event(name, previous, current, uid=0):
    e = types.SimpleNamespace(node_name=name, previous_status=previous, current_status=current)
    if uid:
        e.uid = uid  # Jazzy's log carries uids; Humble's has no such field
    return e


def log(*events):
    return types.SimpleNamespace(event_log=list(events))


class FakeTimer:
    def __init__(self, callback):
        self.callback = callback
        self.running = True

    def cancel(self):
        self.running = False

    def reset(self):
        self.running = True


class FakeAgent(Nav2BtMonitorMixin):
    def __init__(self, tmp_path):
        to_pose = tmp_path / 'to_pose.xml'
        to_pose.write_text(NAV_TO_POSE)
        through = tmp_path / 'through.xml'
        through.write_text(THROUGH_POSES)
        self._node_parameter_cache = {'/bt_navigator': {
            'default_nav_to_pose_bt_xml': str(to_pose),
            'default_nav_through_poses_bt_xml': str(through),
        }}
        self._cached_bt_tree_event = None
        self._nav2_bt_monitor_initialized = True
        self._nav2_bt_navigator = '/bt_navigator'
        self._nav2_bt_publisher_active = False
        self._nav2_bt_trees = {}
        self._nav2_bt_active = None
        self._nav2_bt_pending = []
        self._nav2_bt_goal_active = {}
        self._nav2_bt_last_log = 0.0
        self._nav2_bt_rest_timer = None
        self._nav2_bt_rest_due = None
        self._nav2_bt_unmatched_logged = False
        self._nav2_bt_unreadable = set()
        self._nav2_bt_default_paths = {}
        self._nav2_bt_flush_timer = FakeTimer(self._flush_nav2_bt_changes)
        self.sent = []
        self.timers = []

    def _enqueue(self, payload):
        self.sent.append(payload)

    def create_timer(self, period, callback):
        timer = FakeTimer(callback)
        self.timers.append(timer)
        return timer

    def destroy_timer(self, timer):
        timer.running = False

    def get_logger(self):
        return types.SimpleNamespace(info=lambda *_: None, warning=lambda *_: None,
                                     error=lambda *_: None, debug=lambda *_: None)

    def statuses(self, kind):
        return [e for e in self.sent if e['type'] == kind]

    def changes(self):
        return [(c['uid'], c['name'], c['previous_status'], c['status'])
                for e in self.statuses('bt_status') for c in e['changes']]


def test_nodes_are_numbered_as_bt_cpp_numbers_them():
    nodes = parse_bt_xml(NAV_TO_POSE)['nodes']
    assert [(n['uid'], n['name']) for n in nodes[:8]] == [
        (1, 'NavigateRecovery'), (2, 'NavigateWithReplanning'), (3, 'ControllerSelector'),
        (4, 'PlannerSelector'), (5, 'RateController'), (6, 'ComputePathToPose'),
        (7, 'ComputePathToPose'), (8, 'Sequence'),
    ]
    assert len(nodes) == 28
    # A subtree's nodes come right after its SubTree node, once per instance.
    sub = parse_bt_xml(WITH_SUBTREE)
    assert [(n['uid'], n['name']) for n in sub['nodes']] == [
        (1, 'Root'), (2, 'First'), (3, 'PartSeq'), (4, 'Turn'), (5, 'Pause'),
        (6, 'Second'), (7, 'PartSeq'), (8, 'Turn'),
    ]
    assert sub['structure']['children'][0]['children'][0]['name'] == 'PartSeq'


def test_every_change_goes_out_in_order_on_the_node_the_log_names(tmp_path):
    agent = FakeAgent(tmp_path)
    agent._on_nav2_bt_publisher()
    assert agent.statuses('bt_tree')[0]['tree']['tree_name'] == 'NavigateToPose'
    agent._on_nav2_bt_log(log(
        event('NavigateRecovery', 'IDLE', 'RUNNING', 1),
        event('ComputePathToPose', 'IDLE', 'RUNNING', 6),
        event('ComputePathToPose', 'IDLE', 'RUNNING', 7),
        event('ComputePathToPose', 'RUNNING', 'SUCCESS', 7),
        event('ComputePathToPose', 'SUCCESS', 'IDLE', 7),
    ))
    assert agent.statuses('bt_status') == [], 'changes wait for the flush'
    assert agent._nav2_bt_flush_timer.running
    agent._flush_nav2_bt_changes()
    assert agent.changes() == [
        (1, 'NavigateRecovery', 'IDLE', 'RUNNING'),
        (6, 'ComputePathToPose', 'IDLE', 'RUNNING'),
        (7, 'ComputePathToPose', 'IDLE', 'RUNNING'),
        (7, 'ComputePathToPose', 'RUNNING', 'SUCCESS'),
        (7, 'ComputePathToPose', 'SUCCESS', 'IDLE'),
    ]
    assert not agent._nav2_bt_flush_timer.running
    tree = agent._nav2_bt_active
    assert tree.statuses[6] == 'RUNNING' and tree.statuses[7] == 'IDLE_WAS_SUCCESS'


def test_a_log_without_uids_tells_same_named_nodes_apart_by_their_status(tmp_path):
    agent = FakeAgent(tmp_path)
    agent._on_nav2_bt_publisher()
    agent._on_nav2_bt_log(log(
        event('ComputePathToPose', 'IDLE', 'RUNNING'),      # the RecoveryNode starts first
        event('ComputePathToPose', 'IDLE', 'RUNNING'),      # then its child
        event('ComputePathToPose', 'RUNNING', 'SUCCESS'),   # the child finishes first
        event('ComputePathToPose', 'RUNNING', 'SUCCESS'),
        event('ComputePathToPose', 'SUCCESS', 'IDLE'),      # and is reset first
    ))
    agent._flush_nav2_bt_changes()
    assert [(uid, status) for uid, _, _, status in agent.changes()] == [
        (6, 'RUNNING'), (7, 'RUNNING'), (7, 'SUCCESS'), (6, 'SUCCESS'), (7, 'IDLE'),
    ]


def test_a_log_without_uids_puts_a_change_under_a_running_parent(tmp_path):
    agent = FakeAgent(tmp_path)
    agent._on_nav2_bt_publisher()
    # Planning failed; the recovery branch's Sequence (16, under the running
    # root) starts, not the idle one under ComputePathToPose (8).
    agent._on_nav2_bt_log(log(
        event('NavigateRecovery', 'IDLE', 'RUNNING'),
        event('NavigateWithReplanning', 'IDLE', 'RUNNING'),
        event('NavigateWithReplanning', 'RUNNING', 'FAILURE'),
        event('Sequence', 'IDLE', 'RUNNING'),
        event('Fallback', 'IDLE', 'RUNNING'),
        event('WouldAControllerRecoveryHelp', 'IDLE', 'FAILURE'),
    ))
    agent._flush_nav2_bt_changes()
    assert [uid for uid, *_ in agent.changes()][3:] == [16, 17, 18]


def test_the_tree_rests_when_its_root_finishes(tmp_path):
    agent = FakeAgent(tmp_path)
    agent._on_nav2_bt_publisher()
    agent._on_nav2_bt_log(log(event('NavigateRecovery', 'IDLE', 'RUNNING', 1), event('FollowPath', 'IDLE', 'RUNNING', 12)))
    agent._on_nav2_bt_log(log(event('NavigateRecovery', 'RUNNING', 'FAILURE', 1)))
    # Sent at once, then the tree at rest: Nav2 halts it right away, but only
    # logs that when the navigator's next goal starts.
    assert [c[3] for c in agent.changes()] == ['RUNNING', 'RUNNING', 'FAILURE']
    rest = agent.statuses('bt_tree')[-1]
    status = {n['uid']: n['status'] for n in rest['nodes']}
    assert status[1] == 'IDLE_WAS_FAILURE' and status[12] == 'IDLE_WAS_RUNNING' and status[2] == 'IDLE'


def test_the_other_navigators_log_shows_its_own_tree(tmp_path):
    agent = FakeAgent(tmp_path)
    agent._on_nav2_bt_publisher()
    agent._on_nav2_goal_status('navigate_through_poses', types.SimpleNamespace(status_list=[types.SimpleNamespace(status=2)]))
    agent._on_nav2_bt_log(log(event('NavigateRecovery', 'IDLE', 'RUNNING', 1),
                              event('ComputePathThroughPoses', 'IDLE', 'RUNNING', 6)))
    trees = agent.statuses('bt_tree')
    assert trees[-1]['tree']['tree_name'] == 'NavigateThroughPoses'
    agent._flush_nav2_bt_changes()
    assert agent.statuses('bt_status')[-1]['tree_id'] == trees[-1]['tree_id']
    # Both trees start with the same five nodes; a goal running decides.
    agent._on_nav2_bt_log(log(event('NavigateRecovery', 'RUNNING', 'SUCCESS', 1)))
    assert [t['tree']['tree_name'] for t in agent.statuses('bt_tree')] == [
        'NavigateToPose', 'NavigateThroughPoses', 'NavigateThroughPoses']


def test_the_end_of_one_goal_and_the_start_of_the_next_in_one_message_leave_the_tree_running(tmp_path):
    agent = FakeAgent(tmp_path)
    agent._on_nav2_bt_publisher()
    agent._on_nav2_bt_log(log(event('NavigateRecovery', 'IDLE', 'RUNNING', 1)))
    trees = len(agent.statuses('bt_tree'))
    # Jazzy logs a goal's last tick when the next goal starts.
    agent._on_nav2_bt_log(log(
        event('NavigateRecovery', 'RUNNING', 'SUCCESS', 1),
        event('NavigateRecovery', 'SUCCESS', 'IDLE', 1),
        event('NavigateRecovery', 'IDLE', 'RUNNING', 1),
    ))
    assert len(agent.statuses('bt_tree')) == trees
    assert agent._nav2_bt_active.statuses[1] == 'RUNNING'


def test_a_goal_without_its_last_tick_in_the_log_rests_with_the_goals_result(tmp_path):
    agent = FakeAgent(tmp_path)
    agent._on_nav2_bt_publisher()
    agent._on_nav2_goal_status('navigate_to_pose', types.SimpleNamespace(status_list=[types.SimpleNamespace(status=2)]))
    agent._on_nav2_bt_log(log(event('NavigateRecovery', 'IDLE', 'RUNNING', 1), event('FollowPath', 'IDLE', 'RUNNING', 12)))
    agent._on_nav2_goal_status('navigate_to_pose', types.SimpleNamespace(status_list=[types.SimpleNamespace(status=4)]))
    agent._nav2_bt_last_log -= 10
    agent.timers[-1].callback()
    status = {n['uid']: n['status'] for n in agent.statuses('bt_tree')[-1]['nodes']}
    assert status[1] == 'IDLE_WAS_SUCCESS' and status[12] == 'IDLE_WAS_RUNNING'


def test_a_canceled_goal_rests_its_tree_unless_the_log_moved_on(tmp_path):
    agent = FakeAgent(tmp_path)
    agent._on_nav2_bt_publisher()
    running = types.SimpleNamespace(status_list=[types.SimpleNamespace(status=2)])
    canceled = types.SimpleNamespace(status_list=[types.SimpleNamespace(status=5)])
    agent._on_nav2_goal_status('navigate_to_pose', running)
    agent._on_nav2_bt_log(log(event('NavigateRecovery', 'IDLE', 'RUNNING', 1)))
    agent._on_nav2_goal_status('navigate_to_pose', canceled)
    agent._nav2_bt_last_log -= 10  # the log was quiet since
    agent.timers[-1].callback()
    status = {n['uid']: n['status'] for n in agent.statuses('bt_tree')[-1]['nodes']}
    assert status[1] == 'IDLE_WAS_RUNNING'

    trees = len(agent.statuses('bt_tree'))
    agent._on_nav2_goal_status('navigate_to_pose', running)
    agent._on_nav2_bt_log(log(event('NavigateRecovery', 'IDLE', 'RUNNING', 1)))
    agent._on_nav2_goal_status('navigate_to_pose', canceled)
    agent._on_nav2_bt_log(log(event('NavigateRecovery', 'IDLE', 'RUNNING', 1)))  # a new goal's first tick
    agent.timers[-1].callback()
    assert len(agent.statuses('bt_tree')) == trees


def test_a_page_that_connects_gets_the_tree_as_it_is_now(tmp_path):
    agent = FakeAgent(tmp_path)
    agent._on_nav2_bt_publisher()
    agent._on_nav2_bt_log(log(event('NavigateRecovery', 'IDLE', 'RUNNING', 1)))
    state = agent._build_startup_bt_state_event()
    assert state['type'] == 'bt_state'
    assert {n['uid']: n['status'] for n in state['nodes']}[1] == 'RUNNING'


def test_a_log_from_an_unknown_tree_is_left_out(tmp_path):
    agent = FakeAgent(tmp_path)
    agent._on_nav2_bt_publisher()
    agent._on_nav2_bt_log(log(event('SomethingElse', 'IDLE', 'RUNNING', 3)))
    agent._flush_nav2_bt_changes()
    assert agent.statuses('bt_status') == []
