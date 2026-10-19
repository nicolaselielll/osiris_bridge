"""Graph watcher launch: finds a graph_watcher binary that runs on this machine and starts it."""

import importlib.resources
import os
import platform
import shutil
import stat
import subprocess
import sys

# The bundled build without a distro suffix, graph_watcher_{arch}, is built
# on this distro (the CI builds it there as the fallback for any distro).
UNSUFFIXED_BUILD_DISTRO = 'humble'

# How long a candidate has to stay up to count as started.
LAUNCH_GRACE_S = 0.5


def _is_elf(path):
    # Reject non-ELF binaries (e.g. a macOS Mach-O build packaged by
    # mistake) before trying to run them.
    try:
        with open(path, 'rb') as f:
            return f.read(4) == b'\x7fELF'
    except Exception:
        return False


def _try_launch(path):
    """Spawn a candidate binary and verify it doesn't die immediately.
    An ABI/symbol mismatch against whatever rclcpp is actually loaded
    (e.g. `symbol lookup error: undefined symbol: ...`) surfaces as an
    exit within milliseconds, so a short grace period is enough to tell
    a working binary from a broken one. Returns (process, None) for a
    binary still running after the grace period, or (None, why it failed)
    as soon as it exits."""
    try:
        os.chmod(path, os.stat(path).st_mode | stat.S_IXUSR | stat.S_IXGRP | stat.S_IXOTH)
        proc = subprocess.Popen([path], stdout=subprocess.DEVNULL, stderr=subprocess.PIPE)
    except OSError as e:
        return None, f'{path} could not be started: {e}'
    try:
        rc = proc.wait(timeout=LAUNCH_GRACE_S)
    except subprocess.TimeoutExpired:
        return proc, None
    try:
        err = proc.stderr.read().decode(errors='replace').strip()
    except Exception:
        err = ''
    return None, f'{path} exited immediately (rc={rc})' + (f': {err}' if err else '')


def _candidate_names(arch, distro, bundled_names):
    """The bundled binaries to try, in order:
      a. the build for this ROS_DISTRO: graph_watcher_{arch}_{distro}, or on
         Humble the unsuffixed graph_watcher_{arch}, which is built there
      b. every other distro's build for this arch, for an unset ROS_DISTRO
         or one with no build of its own, in case one happens to be
         ABI-compatible
      c. the unsuffixed graph_watcher_{arch}, the last-resort fallback
      d. graph_watcher (legacy / colcon-installed generic name)
    """
    unsuffixed = f'graph_watcher_{arch}'
    names = []
    if distro == UNSUFFIXED_BUILD_DISTRO:
        names.append(unsuffixed)
    if distro:
        names.append(f'{unsuffixed}_{distro}')
    names += sorted(n for n in bundled_names if n.startswith(f'{unsuffixed}_'))
    names += [unsuffixed, 'graph_watcher']
    return list(dict.fromkeys(names))


def _launch_bundled(bin_dir, arch, distro, failures):
    """Launches the first bundled binary that starts, trying them in the
    order of _candidate_names. Returns (path, process), or (None, None) if
    none starts. Each failure is added to failures."""
    try:
        bundled_names = [entry.name for entry in bin_dir.iterdir()]
    except Exception:
        bundled_names = []
    for name in _candidate_names(arch, distro, bundled_names):
        candidate = bin_dir.joinpath(name)
        if not candidate.is_file():
            continue
        path = str(candidate)
        if not _is_elf(path):
            continue
        proc, failure = _try_launch(path)
        if proc is not None:
            return path, proc
        failures.append(failure)
    return None, None


def forward_stderr(stream, log):
    """Passes each line of a started graph_watcher's stderr to log, for as
    long as it runs. The reading is what keeps the pipe from filling: a
    graph_watcher whose stderr pipe is full blocks on its next write and
    stops reporting graph changes. So a line that can't be decoded or
    logged is skipped, never allowed to end the loop."""
    for raw in stream:
        try:
            line = raw.decode(errors='replace').rstrip()
            # rclcpp prints these on SIGINT/SIGTERM — informational noise.
            if line and 'signal_handler(' not in line:
                log(line)
        except Exception:
            pass


def start_graph_watcher():
    """Starts graph_watcher and returns its process, or None if no candidate binary could be started."""
    # A graph_watcher on PATH (a colcon dev workspace with
    # install/setup.bash sourced) comes first, then the bundled binaries.
    # Candidates that fail are only reported when none starts: on a working
    # robot, a build for another distro failing first is expected.
    _watcher_proc = None
    _failures = []
    _watcher_bin = shutil.which('graph_watcher')

    if _watcher_bin is not None:
        if sys.platform != 'linux':
            import logging
            logging.getLogger(__name__).warning(
                f"graph_watcher is a Linux binary and cannot run on {sys.platform} — "
                "graph events will not be available."
            )
            _watcher_bin = None
        elif not _is_elf(_watcher_bin):
            import logging
            logging.getLogger(__name__).error(
                f"osiris_graph_watcher binary at '{_watcher_bin}' is not a Linux ELF "
                "(wrong platform — was a macOS binary published by mistake?). "
                "Graph events will not be available."
            )
            _watcher_bin = None
        else:
            _watcher_proc, _failure = _try_launch(_watcher_bin)
            if _watcher_proc is None:
                _failures.append(_failure)
                _watcher_bin = None

    if _watcher_bin is None and sys.platform == 'linux':
        try:
            _watcher_bin, _watcher_proc = _launch_bundled(
                importlib.resources.files('osiris_agent').joinpath('bin'),
                platform.machine(),  # 'x86_64' or 'aarch64'
                os.environ.get('ROS_DISTRO', ''),  # e.g. 'humble', 'jazzy', 'lyrical'
                _failures,
            )
        except Exception:
            _watcher_bin = None

    if _watcher_proc is not None:
        print(f'[osiris] graph_watcher started: {_watcher_bin} (pid={_watcher_proc.pid})', flush=True)
    else:
        import logging
        logging.getLogger(__name__).warning(
            "osiris_graph_watcher not found or failed to start — graph events will not be available."
            + ''.join(f'\n  {failure}' for failure in _failures)
        )
    return _watcher_proc
