"""A request from Osiris reads nothing on the robot but finished ROS 2 bags in
its bag folder: the list shows only those, and a download zips only one of
them, never through a symlink."""

import io
import os
import threading
import types
import zipfile

import pytest

from osiris_agent import bag_recorder
from osiris_agent.bag_recorder import BagRecorderMixin, bag_files, bag_folder


def make_bag(folder, files=('bag_0.mcap',)):
    os.makedirs(folder)
    with open(os.path.join(folder, 'metadata.yaml'), 'w') as f:
        f.write('rosbag2_bagfile_information: {}\n')
    for name in files:
        with open(os.path.join(folder, name), 'wb') as f:
            f.write(b'x' * 10)
    return str(folder)


@pytest.fixture
def robot(tmp_path):
    """A robot's home folder with a bag folder in it, and a secret beside it."""
    home = tmp_path / 'home'
    bags = home / 'ros2_bags'
    bags.mkdir(parents=True)
    (home / '.ssh').mkdir()
    (home / '.ssh' / 'id_ed25519').write_text('PRIVATE KEY')
    return types.SimpleNamespace(home=str(home), bags=str(bags))


def test_a_finished_bag_directly_in_the_bag_folder_is_one(robot):
    bag = make_bag(os.path.join(robot.bags, 'bag_1792000000'))
    assert bag_folder(robot.bags, bag) == (bag, None)


@pytest.mark.parametrize('path, why_not', [
    ('{home}/.ssh', 'not directly in bag_output_dir'),
    ('{bags}/../.ssh', 'not directly in bag_output_dir'),
    ('{bags}', 'not directly in bag_output_dir'),
    ('{bags}/bag_1792000000/nested', 'not directly in bag_output_dir'),
    ('{bags}/missing', 'not a folder'),
    ('', 'no path'),
])
def test_paths_outside_or_nested_are_refused(robot, path, why_not):
    make_bag(os.path.join(robot.bags, 'bag_1792000000'))
    make_bag(os.path.join(robot.bags, 'bag_1792000000', 'nested'))
    folder, reason = bag_folder(robot.bags, path.format(home=robot.home, bags=robot.bags))
    assert folder is None
    assert reason == why_not


def test_a_folder_without_metadata_is_not_a_bag(robot):
    os.makedirs(os.path.join(robot.bags, 'notes'))
    assert bag_folder(robot.bags, os.path.join(robot.bags, 'notes')) == (None, 'not a ROS 2 bag (no metadata.yaml)')


def test_a_symlink_to_a_folder_is_refused_even_if_it_holds_a_bag(robot, tmp_path):
    elsewhere = make_bag(os.path.join(str(tmp_path), 'elsewhere'))
    os.symlink(elsewhere, os.path.join(robot.bags, 'linked'))
    os.symlink(robot.home, os.path.join(robot.bags, 'home'))
    assert bag_folder(robot.bags, os.path.join(robot.bags, 'linked')) == (None, 'a symlink')
    assert bag_folder(robot.bags, os.path.join(robot.bags, 'home')) == (None, 'a symlink')


def test_a_symlinked_metadata_file_does_not_make_a_bag(robot):
    folder = os.path.join(robot.bags, 'fake')
    os.makedirs(folder)
    os.symlink(os.path.join(robot.home, '.ssh', 'id_ed25519'), os.path.join(folder, 'metadata.yaml'))
    assert bag_folder(robot.bags, folder) == (None, 'not a ROS 2 bag (no metadata.yaml)')


def test_a_bags_files_never_go_through_a_symlink(robot):
    bag = make_bag(os.path.join(robot.bags, 'bag_1792000000'))
    os.symlink(os.path.join(robot.home, '.ssh', 'id_ed25519'), os.path.join(bag, 'key'))
    os.symlink(os.path.join(robot.home, '.ssh'), os.path.join(bag, 'ssh'))
    assert sorted(os.path.basename(f) for f in bag_files(bag)) == ['bag_0.mcap', 'metadata.yaml']


class FakeUpload:
    """http.client's connection, recording what was uploaded."""

    uploads = []

    def __init__(self, netloc, timeout=None):
        self.netloc = netloc

    def request(self, method, path, body=None, headers=None):
        FakeUpload.uploads.append(body.read())

    def getresponse(self):
        return types.SimpleNamespace(status=200, read=lambda: b'')

    def close(self):
        pass


class FakeAgent(BagRecorderMixin):
    def __init__(self, bag_dir):
        self.bag_dir = bag_dir
        self.errors = []
        self.ws_url = 'ws://gateway.test:8080'
        self._bag_lock = threading.Lock()
        self._bag_output_path = None

    def get_parameter(self, name):
        assert name == 'bag_output_dir'
        return types.SimpleNamespace(get_parameter_value=lambda: types.SimpleNamespace(string_value=self.bag_dir))

    def get_logger(self):
        log = lambda *a, **k: None  # noqa: E731
        return types.SimpleNamespace(info=log, debug=log, warning=log, error=self.errors.append)


@pytest.fixture
def uploads(monkeypatch):
    FakeUpload.uploads = []
    monkeypatch.setattr(bag_recorder.http.client, 'HTTPConnection', FakeUpload)
    return FakeUpload.uploads


def test_the_list_shows_only_finished_bags(robot, tmp_path):
    make_bag(os.path.join(robot.bags, 'bag_1792000000'))
    os.makedirs(os.path.join(robot.bags, 'bag_1792000100'))  # being recorded: no metadata yet
    os.makedirs(os.path.join(robot.bags, '.cache'))
    os.symlink(make_bag(os.path.join(str(tmp_path), 'elsewhere')), os.path.join(robot.bags, 'linked'))
    bags = FakeAgent(robot.bags)._get_bag_files_snapshot()
    assert [(b['name'], b['size_bytes']) for b in bags] == [('bag_1792000000', 10 + 32)]


def test_a_bag_folder_set_to_home_lists_none_of_its_folders(robot):
    assert FakeAgent(robot.home)._get_bag_files_snapshot() == []


def test_a_download_zips_the_bag_and_nothing_it_links_to(robot, uploads):
    bag = make_bag(os.path.join(robot.bags, 'bag_1792000000'))
    os.symlink(os.path.join(robot.home, '.ssh', 'id_ed25519'), os.path.join(bag, 'key'))
    agent = FakeAgent(robot.bags)
    agent._send_bag_download(bag, 'r1', 'http://gateway.test:8080/bag/upload')
    [upload] = uploads
    names = sorted(zipfile.ZipFile(io.BytesIO(upload)).namelist())
    assert names == ['bag_1792000000/bag_0.mcap', 'bag_1792000000/metadata.yaml']
    assert agent.errors == []


@pytest.mark.parametrize('path', [
    '{home}/.ssh',                       # outside the bag folder
    '{bags}/../.ssh',                    # outside, by a relative step
    '{bags}/notes',                      # not a bag
    '{bags}/bag_1792000000/nested',      # nested
    '{bags}/linked',                     # a symlink to a bag elsewhere
])
def test_a_download_of_anything_else_is_refused(robot, tmp_path, uploads, path):
    make_bag(os.path.join(robot.bags, 'bag_1792000000'))
    make_bag(os.path.join(robot.bags, 'bag_1792000000', 'nested'))
    os.makedirs(os.path.join(robot.bags, 'notes'))
    os.symlink(make_bag(os.path.join(str(tmp_path), 'elsewhere')), os.path.join(robot.bags, 'linked'))
    agent = FakeAgent(robot.bags)
    agent._send_bag_download(path.format(home=robot.home, bags=robot.bags), 'r1', 'http://gateway.test:8080/bag/upload')
    assert uploads == []
    [error] = agent.errors
    assert error.startswith('[bag] download refused:')
