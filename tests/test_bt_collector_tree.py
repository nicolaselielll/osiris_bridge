"""A BT.CPP tree from Groot2 reaches the pane whole, subtrees included, and
each recorded transition says where its node came from."""

import struct

from osiris_agent.bt_collector import BTCollector, BTTree

# As BT.CPP 4.10's Groot2 publisher sends it: every subtree instance as a
# BehaviorTree of its own after the main tree, matched by _fullpath.
GROOT2_XML = """<root BTCPP_format="4">
    <BehaviorTree ID="Mission" _fullpath="">
        <Sequence name="Mission" _uid="1">
            <Work name="MoveTo" _uid="2" ms="1500"/>
            <SubTree ID="Inspect" _fullpath="InspectLeft" _uid="3"/>
            <SubTree ID="Inspect" _fullpath="InspectRight" _uid="6"/>
        </Sequence>
    </BehaviorTree>
    <BehaviorTree ID="Inspect" _fullpath="InspectLeft">
        <Sequence name="InspectSeq" _uid="4">
            <Work name="Scan" _uid="5"/>
        </Sequence>
    </BehaviorTree>
    <BehaviorTree ID="Inspect" _fullpath="InspectRight">
        <Sequence name="InspectSeq" _uid="7">
            <Work name="Scan" _uid="8"/>
        </Sequence>
    </BehaviorTree>
    <TreeNodesModel>
        <SubTree ID="SubTree"/>
    </TreeNodesModel>
</root>"""


def collector():
    events = []
    c = BTCollector(events.append)
    c._log_info = c._log_debug = c._log_warn = c._log_error = lambda *_: None
    tree = BTTree(tree_id='t1')
    tree.nodes_list = c._parse_xml_tree(GROOT2_XML, tree)
    c._current_tree = tree
    return c, events


def walk(node, out):
    out.append((node.get('uid'), node['name']))
    for child in node.get('children') or []:
        walk(child, out)
    return out


class FakeSocket:
    def __init__(self, *replies):
        self.replies = list(replies)

    def send(self, *_args, **_kwargs):
        pass

    def recv_multipart(self):
        return self.replies.pop(0)


def transitions(*items):
    data = b''
    for usec, uid, status in items:
        data += usec.to_bytes(6, 'little') + struct.pack('<H', uid) + bytes([status])
    return [b'h' * 22, data]


def test_subtrees_sit_under_their_subtree_node_with_its_name():
    c, _ = collector()
    structure = c._current_tree.structure
    assert structure['tree_name'] == 'Mission'
    assert walk(structure, []) == [
        (1, 'Mission'), (2, 'MoveTo'),
        (3, 'InspectLeft'), (4, 'InspectSeq'), (5, 'Scan'),
        (6, 'InspectRight'), (7, 'InspectSeq'), (8, 'Scan'),
    ]
    assert {n['uid']: n['name'] for n in c._current_tree.nodes_list}[3] == 'InspectLeft'


def test_each_transition_comes_from_the_status_the_last_one_left():
    c, events = collector()
    c._recording = True
    # RUNNING, SUCCESS, then IDLE: BT.CPP sends IDLE either as 0 or as 10 + previous.
    c._poll_transitions(FakeSocket(transitions((1, 2, 1), (2, 2, 2), (3, 2, 0), (4, 5, 1), (5, 5, 13))))
    changes = [(ch['uid'], ch['previous_status'], ch['status']) for ch in events[-1]['changes']]
    assert changes == [(2, 'IDLE', 'RUNNING'), (2, 'RUNNING', 'SUCCESS'), (2, 'SUCCESS', 'IDLE'),
                       (5, 'IDLE', 'RUNNING'), (5, 'FAILURE', 'IDLE')]
    assert c._last_statuses[2] == 'IDLE_WAS_SUCCESS'


def test_a_snapshot_while_recording_leaves_the_statuses_to_the_transitions():
    c, events = collector()
    c._recording = True
    c._last_statuses = {2: 'RUNNING'}
    # The snapshot already says MoveTo succeeded and went idle; the transitions
    # that get it there come with the next poll.
    tree_id = bytes(range(16))
    c._current_tree.tree_id = tree_id.hex()
    snapshot = [b'h' * 6 + tree_id, struct.pack('<HB', 2, 12)]
    c._poll_status(FakeSocket(snapshot))
    assert c._last_statuses[2] == 'RUNNING' and events == []
    c._poll_transitions(FakeSocket(transitions((1, 2, 2), (2, 2, 0))))
    assert [(ch['previous_status'], ch['status']) for ch in events[-1]['changes']] == [('RUNNING', 'SUCCESS'), ('SUCCESS', 'IDLE')]


def test_a_page_that_connects_gets_the_statuses_as_they_are_now():
    c, _ = collector()
    c._last_statuses = {1: 'RUNNING', 2: 'IDLE_WAS_SUCCESS'}
    event = c.tree_event()
    assert {n['uid']: n['status'] for n in event['nodes']} == {
        1: 'RUNNING', 2: 'IDLE_WAS_SUCCESS', 3: 'IDLE', 4: 'IDLE', 5: 'IDLE', 6: 'IDLE', 7: 'IDLE', 8: 'IDLE'}
