import pytest

from osiris_agent.topic_names import (
    MAX_TOPIC_NAME_LENGTH,
    MAX_TOPICS_PER_REQUEST,
    are_valid_topic_names,
    is_valid_topic_name,
)


@pytest.mark.parametrize('name', [
    '/scan',
    '/tf',
    '/tf_static',
    '/cmd_vel',
    '/robot_1/camera/image_raw',
    '/_action/feedback',
    '/a/b/c/d',
])
def test_valid_topic_names(name):
    assert is_valid_topic_name(name)


@pytest.mark.parametrize('name', [
    '',
    '/',
    'scan',
    '//scan',
    '/scan/',
    '/scan//x',
    '/9lives',
    '/sc an',
    '/scan;rm',
    '/scan\n',
    '~/private',
    '/{ns}/scan',
    '/a-b',
    '--output',
    '-a',
    '--storage',
    '/scan --all',
])
def test_invalid_topic_names(name):
    assert not is_valid_topic_name(name)


@pytest.mark.parametrize('value', [None, 5, 1.5, ['/scan'], {'a': 1}, b'/scan'])
def test_non_string_names_are_invalid(value):
    assert not is_valid_topic_name(value)


def test_name_length_limit():
    assert is_valid_topic_name('/' + 'a' * (MAX_TOPIC_NAME_LENGTH - 1))
    assert not is_valid_topic_name('/' + 'a' * MAX_TOPIC_NAME_LENGTH)


def test_list_validation():
    assert are_valid_topic_names([])
    assert are_valid_topic_names(['/scan', '/tf'])
    assert not are_valid_topic_names(['/scan', '--all'])
    assert not are_valid_topic_names('/scan')
    assert not are_valid_topic_names(None)


def test_list_size_limit():
    assert are_valid_topic_names(['/t'] * MAX_TOPICS_PER_REQUEST)
    assert not are_valid_topic_names(['/t'] * (MAX_TOPICS_PER_REQUEST + 1))
