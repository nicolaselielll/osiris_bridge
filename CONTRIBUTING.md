# Contributing

Thanks for helping improve the Osiris bridge. Bug reports and pull requests are welcome.

## Reporting a bug

Open an issue at https://github.com/nicolaselielll/osiris_bridge/issues with:

- what happened and what you expected,
- how to reproduce it,
- the bridge version (`pip show osiris-bridge`), the ROS 2 distribution, the operating system and the CPU architecture,
- the bridge's output around the problem.

Please report security problems privately instead, as described in [SECURITY.md](SECURITY.md).

## Sending a change

1. Fork the repository and create a branch for your change.
2. Keep each pull request to one fix or feature, and describe what it changes and why.
3. Add or update tests for what you changed.
4. Run the tests (below). CI runs them again on every push and pull request.

Comments in the code should explain what it does and why, for someone reading it later.

### Running the tests

The tests run under ROS 2 Humble. On a machine with ROS 2 Humble installed:

```bash
sudo apt install ros-humble-nav2-msgs
source /opt/ros/humble/setup.bash
pip install -e . pytest
python3 -m pytest -q
```

Tests that need ROS skip themselves where it isn't installed, so `pytest` also runs on a plain Python setup, with fewer tests.

### graph_watcher

`graph_watcher/` holds the source of the C++ helper the bridge starts to watch the ROS graph. The binaries in `osiris_agent/bin/` are built by the `Build graph_watcher binaries` workflow, so don't commit rebuilt binaries in a pull request.

## License

The bridge is licensed under the [Apache License 2.0](LICENSE). By sending a contribution, you agree that it is licensed under the same terms, as section 5 of the license describes.
