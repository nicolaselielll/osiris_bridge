"""graph_watcher's bundled builds are tried best match first, failed ones are only reported when none
starts, and a started one's stderr is read for as long as it runs."""

import io
import logging
import threading
import time
import types

import pytest

from osiris_agent import graph_watcher_launcher as launcher

# Shell scripts stand in for the binaries: one keeps running, the other fails
# the way the Jazzy build does on a Humble robot.
STARTS = '#!/bin/sh\nexec sleep 30\n'
FAILS = "#!/bin/sh\necho \"version 'GLIBCXX_3.4.32' not found\" >&2\nexit 1\n"

AARCH64_BUILDS = ['graph_watcher_aarch64', 'graph_watcher_aarch64_jazzy', 'graph_watcher_aarch64_lyrical']


def test_humble_tries_the_unsuffixed_humble_build_first():
    assert launcher._candidate_names('aarch64', 'humble', AARCH64_BUILDS) == [
        'graph_watcher_aarch64',
        'graph_watcher_aarch64_humble',
        'graph_watcher_aarch64_jazzy',
        'graph_watcher_aarch64_lyrical',
        'graph_watcher',
    ]


def test_jazzy_tries_its_own_build_first_and_the_humble_build_last():
    assert launcher._candidate_names('aarch64', 'jazzy', AARCH64_BUILDS) == [
        'graph_watcher_aarch64_jazzy',
        'graph_watcher_aarch64_lyrical',
        'graph_watcher_aarch64',
        'graph_watcher',
    ]


def test_an_unset_distro_tries_every_suffixed_build_before_the_fallback():
    assert launcher._candidate_names('aarch64', '', AARCH64_BUILDS) == [
        'graph_watcher_aarch64_jazzy',
        'graph_watcher_aarch64_lyrical',
        'graph_watcher_aarch64',
        'graph_watcher',
    ]


@pytest.fixture
def bin_dir(tmp_path, monkeypatch):
    """A Linux aarch64 install with nothing on PATH, whose bundled binaries
    are the stubs a test writes into the returned directory."""
    bin_dir = tmp_path / 'bin'
    bin_dir.mkdir()
    monkeypatch.setattr(launcher, 'shutil', types.SimpleNamespace(which=lambda name: None))
    monkeypatch.setattr(launcher, 'sys', types.SimpleNamespace(platform='linux'))
    monkeypatch.setattr(launcher, 'platform', types.SimpleNamespace(machine=lambda: 'aarch64'))
    monkeypatch.setattr(launcher, 'importlib', types.SimpleNamespace(
        resources=types.SimpleNamespace(files=lambda package: tmp_path)))
    monkeypatch.setattr(launcher, '_is_elf', lambda path: True)
    # Long enough that a stub which fails is seen exiting even when the OS is
    # slow to start a freshly written script; only a stub that keeps running
    # waits it out.
    monkeypatch.setattr(launcher, 'LAUNCH_GRACE_S', 2.0)
    return bin_dir


def write_stubs(bin_dir, **scripts):
    for name, script in scripts.items():
        path = bin_dir / name
        path.write_text(script)
        path.chmod(0o755)


def printed(capsys):
    # A failed candidate used to be printed as it happened. Warnings go
    # through logging and are checked with the warnings fixture.
    return capsys.readouterr().out


class _Collect(logging.Handler):
    def __init__(self):
        super().__init__(logging.WARNING)
        self.records = []

    def emit(self, record):
        self.records.append(record)


@pytest.fixture
def warnings():
    """The launcher's warnings, collected by a handler on its own logger:
    caplog captures nothing under the pytest that comes with ROS 2 Humble."""
    handler = _Collect()
    logger = logging.getLogger(launcher.__name__)
    logger.addHandler(handler)
    yield handler.records
    logger.removeHandler(handler)


@pytest.fixture
def started():
    procs = []
    yield procs
    for proc in procs:
        if proc is not None:
            proc.kill()
            proc.wait()


def test_on_humble_the_humble_build_starts_without_trying_the_others(bin_dir, started, monkeypatch, capsys, warnings):
    write_stubs(bin_dir, graph_watcher_aarch64=STARTS,
                graph_watcher_aarch64_jazzy=FAILS, graph_watcher_aarch64_lyrical=FAILS)
    monkeypatch.setenv('ROS_DISTRO', 'humble')

    proc = launcher.start_graph_watcher()
    started.append(proc)

    assert proc.args == [str(bin_dir / 'graph_watcher_aarch64')]
    assert 'GLIBCXX' not in printed(capsys)
    assert warnings == []


def test_a_failed_candidate_isnt_reported_when_a_later_one_starts(bin_dir, started, monkeypatch, capsys, warnings):
    write_stubs(bin_dir, graph_watcher_aarch64=STARTS,
                graph_watcher_aarch64_jazzy=FAILS, graph_watcher_aarch64_lyrical=FAILS)
    monkeypatch.delenv('ROS_DISTRO', raising=False)

    proc = launcher.start_graph_watcher()
    started.append(proc)

    assert proc.args == [str(bin_dir / 'graph_watcher_aarch64')]
    assert 'GLIBCXX' not in printed(capsys)
    assert warnings == []


def test_every_failure_is_listed_when_nothing_starts(bin_dir, started, monkeypatch, capsys, warnings):
    write_stubs(bin_dir, graph_watcher_aarch64=FAILS,
                graph_watcher_aarch64_jazzy=FAILS, graph_watcher_aarch64_lyrical=FAILS)
    monkeypatch.setenv('ROS_DISTRO', 'humble')

    proc = launcher.start_graph_watcher()
    started.append(proc)

    assert proc is None
    assert 'GLIBCXX' not in printed(capsys)
    [warning] = warnings
    lines = warning.getMessage().splitlines()
    assert 'not found or failed to start' in lines[0]
    assert lines[1:] == [
        f"  {bin_dir / name} exited immediately (rc=1): version 'GLIBCXX_3.4.32' not found"
        for name in ('graph_watcher_aarch64', 'graph_watcher_aarch64_jazzy', 'graph_watcher_aarch64_lyrical')
    ]


def test_reading_goes_on_past_a_line_that_isnt_utf8():
    stream = io.BytesIO(b"bytes that aren't UTF-8: \xff\xfe\nafter\n")
    lines = []
    launcher.forward_stderr(stream, lines.append)
    assert lines == ["bytes that aren't UTF-8: ��", 'after']


def test_reading_goes_on_when_logging_a_line_fails():
    stream = io.BytesIO(b'first\nsecond\n')
    seen = []

    def log(line):
        seen.append(line)
        raise RuntimeError('logger is gone')

    launcher.forward_stderr(stream, log)
    assert seen == ['first', 'second']


# Writes a line that isn't UTF-8, then 2000 lines of 100 bytes, three times
# what a pipe holds, then marks that every write went through.
FLOODS = """#!/bin/sh
printf 'not UTF-8: \\377\\376\\n' >&2
for i in $(seq 2000); do echo "$LINE" >&2; done
touch "$FLOOD_MARK"
exec sleep 30
"""


def test_a_graph_watcher_flooding_its_stderr_is_never_blocked(tmp_path, started, monkeypatch):
    stub = tmp_path / 'graph_watcher'
    stub.write_text(FLOODS)
    stub.chmod(0o755)
    mark = tmp_path / 'flood-done'
    monkeypatch.setenv('FLOOD_MARK', str(mark))
    monkeypatch.setenv('LINE', 'x' * 99)
    # Until something reads, the stub is blocked on a full pipe, so it is
    # still up when the grace period ends.
    monkeypatch.setattr(launcher, 'LAUNCH_GRACE_S', 2.0)
    proc, failure = launcher._try_launch(str(stub))
    started.append(proc)
    assert failure is None

    lines = []
    threading.Thread(target=launcher.forward_stderr, args=(proc.stderr, lines.append), daemon=True).start()
    deadline = time.monotonic() + 20
    while (not mark.exists() or len(lines) < 2001) and time.monotonic() < deadline:
        time.sleep(0.05)

    assert mark.exists()
    assert len(lines) == 2001
    assert lines[0] == 'not UTF-8: ��'
