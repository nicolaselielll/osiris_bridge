"""Validation for ROS topic names received from the gateway."""

import re

MAX_TOPIC_NAME_LENGTH = 256
MAX_TOPICS_PER_REQUEST = 200

# A fully qualified ROS 2 topic name: one or more `/token` segments, where a
# token starts with a letter or underscore and continues with letters, digits
# or underscores. This rules out relative names, substitutions, repeated or
# trailing slashes, and anything that could be read as a command-line flag.
_TOPIC_NAME_RE = re.compile(r'^(?:/[A-Za-z_][A-Za-z0-9_]*)+$')


def is_valid_topic_name(name):
    return (
        isinstance(name, str)
        and len(name) <= MAX_TOPIC_NAME_LENGTH
        and _TOPIC_NAME_RE.fullmatch(name) is not None
    )


def are_valid_topic_names(names):
    """True if `names` is a list of at most MAX_TOPICS_PER_REQUEST valid names."""
    return (
        isinstance(names, list)
        and len(names) <= MAX_TOPICS_PER_REQUEST
        and all(is_valid_topic_name(name) for name in names)
    )
