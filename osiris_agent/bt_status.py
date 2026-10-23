"""Behavior tree node statuses as the bridge reports them: BT.CPP's names, and
its encoding for a node at rest that remembers how it last ended."""


def resting(status: str) -> str:
    """A node's status once it goes idle: IDLE_WAS_SUCCESS after a success, and
    so on, as BT.CPP's Groot2 publisher reports it."""
    if status in ('RUNNING', 'SUCCESS', 'FAILURE'):
        return f'IDLE_WAS_{status}'
    return status


def plain(status: str) -> str:
    """The status without the memory of a resting node: IDLE_WAS_SUCCESS is IDLE."""
    return 'IDLE' if status.startswith('IDLE') else status
