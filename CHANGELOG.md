# Changelog

## 0.5.1 (2026-10-22)

### Fixes

- **TF frames on a simulated robot are no longer all stale.** A robot running a simulation stamps its transforms with simulation time, which counts from the simulation's start. The bridge measured their age against the wall clock, so every moving frame showed as stale, with an age of about 56 years. When every transform is stamped before 2001, ages are now measured from the simulation's own clock (/clock), or from the newest transform until /clock arrives. Robots on the wall clock are unchanged.
- **The graph watcher is started again if it exits.** Without it the bridge heard of no graph change, and the app kept showing the robot's last graph until the bridge restarted. The bridge now checks it every 5 seconds, starts it again, and reads the graph once to catch up on what changed meanwhile. One that keeps crashing is retried once a minute after three quick restarts.
- **Ctrl+C no longer prints a stack trace now and then.** On a busy robot, ROS could shut down while the bridge was waiting for events, and the bridge reported that as an error.

### Packaging

- The source package now includes the graph watcher's C++ source, which the bundled graph_watcher binaries are built from.

## 0.5.0 (2026-10-19)

### When you upgrade

- **The package is now `osiris-bridge`.** Remove `osiris-agent` first, since both install the same files: `pip3 uninstall -y osiris-agent && pip3 install osiris-bridge` (on Ubuntu 24.04 and newer, add `--break-system-packages` to both). The `osiris` command, your token and your settings stay the same.

### Documentation

- This program is now called the Osiris bridge, as it is in the Osiris app (robot menu > Bridge opens *Configure bridge*, which used to be *Configure agent*). The `osiris` command and every parameter keep their names.
- The README is rewritten: requirements, connecting a robot (the token and `OSIRIS_WS_URL` from *Configure bridge*), what the bridge sends, the two configuration modes, every parameter with its default, environment variables, behavior trees, robot commands, bags, several robots on one ROS network, and troubleshooting.
- The package description, CONTRIBUTING and SECURITY call it the bridge.

## 0.4.0 (2026-10-16)

Changes since 0.3.123.

### When you upgrade

- **Upgrade to use Drive and Spin.** Osiris sends a motion command only to an agent that reports commands as on, which this version does and earlier ones don't. Until a robot runs this version, every spin or drive sent to it is refused.
- **Motion commands are off until an owner allows them.** 0.3.123 ran every spin or drive Osiris sent. Now an owner of the robot's workspace turns them on with *Allow robot commands* in *Configure agent* in the Osiris app, and every command must stay within the limits below. A ROS 2 parameters file on the robot can turn commands off or set stricter limits, but it can't turn them on while they're off in the app. If your parameters file sets `commands_enabled: true`, turn on *Allow robot commands* as well.
- **The bag folder is set on the robot only.** The app can no longer change `bag_output_dir`. If you had set a folder there, recordings go back to the robot's own `bag_output_dir` (default `~/ros2_bags`). To keep your folder, set it on the robot: `osiris --ros-args -p bag_output_dir:=/data/bags`. Setting only `bag_output_dir`, this way or in a parameters file, keeps the configuration from the app in effect.
- **The Bags pane lists only finished recordings directly in the bag folder**, meaning folders with a `metadata.yaml`. A recording stopped before `ros2 bag record` finished it has none and no longer shows.
- **On Ubuntu 24.04 and newer** (ROS 2 Jazzy and later), install with `pip3 install --break-system-packages osiris-agent`.

### Robot commands

- `commands_enabled` (default `false`) and four limits, checked before a goal reaches Nav2. Only a literal `true` turns commands on, and they stay off until the agent's first configuration is applied:

  | Limit | Default |
  |---|---|
  | `command_max_speed_mps` | 0.5 m/s |
  | `command_max_distance_m` | 5 m |
  | `command_max_angle_deg` | 360° |
  | `command_max_time_allowance_s` | 120 s |

- A field a command leaves out is checked at the value the agent would use: a drive's speed is 0.15 m/s, and the time allowance is 60 s. A value that isn't a finite number is refused, and so is a speed or time allowance of 0 or less.
- A refusal now comes with a message that names the field as it was sent, the value and the limit, for example "angle_deg 400 is over this robot's limit of 360 (command_max_angle_deg)". The reasons themselves (`limit_exceeded:<field>`, `invalid_params:<field>`) are unchanged.
- The agent reports the settings it runs with, so the app shows which limits apply.
- Cancelling a running command always works.
- A backward drive (negative distance) works. It used to be rejected by Nav2.
- A spin or drive is refused with `motion_in_progress:<action>` while another goal moves the robot, such as a navigation started from the app or RViz.
- A second command while the agent's own is still running is refused with `command_already_in_progress`.
- Commands go to the Nav2 in the agent's own namespace, or to the only Nav2 in the graph (for example one under `/robot1`). When several namespaces have one and none is the agent's, they're refused with `action_server_ambiguous`.

### Bags

- The list and downloads cover only folders directly in `bag_output_dir` that hold a finished bag. A download never follows a symlink.
- A recording asks only for fully qualified topic names: at most 200 topics of up to 256 characters each.
- When a recording stops, the agent sends the bag list again, so the new bag shows after a reload and on other computers.

### Connection

- After it starts, the agent connects once ROS discovery has settled: when no node or topic has changed for 4 s, or at most 15 s after the start. Before, a restart could report a partial graph, which showed in the robot's history as Nav2 going down and coming back. Reconnecting later doesn't wait.
- When the gateway refuses the connection, the log says why and what to do: the workspace's storage is full, the storage check failed, or the token is missing, invalid or was rotated.
- The wait between reconnect attempts starts over after a connection that stayed up, instead of staying at 30 s. Every wait has some jitter.
- After a dropped connection, a topic's binary data no longer arrives without its header on the next one. A drop no longer prints an asyncio traceback.

### Monitoring

- **Topics:** best-effort topics (Gazebo sensors, many LiDAR drivers) and latched ones (`/map`, AMCL's pose while the robot stands still) are now read. Subscriptions match the publishers' QoS and follow them when they change.
- **Namespaced Nav2** (`nav2_bringup`'s `use_namespace`): the behavior tree monitor and TF follow a Nav2 under a namespace, and switch when one comes up later under another.
- **TF:** static transforms and the root frame are no longer reported as stale. Static transforms are marked `is_static`.
- **Graph:**
  - publishers and subscribers that running nodes add or remove are noticed
  - a node changing a parameter all the time no longer holds up graph updates
  - an action's status topic that only the agent reads no longer shows as an action without a server
- **Parameters:** a node's parameters declared while the agent fetches them, as Nav2's nodes do while configuring, are no longer missing.
- **ros2_control:** controllers and hardware are polled on their own timer, every `ros2_control_poll_interval`, so a state change shows without a graph change.
- **Goals:** goals that had already finished when the agent (re)started are marked `replayed`, instead of showing as goals that just ended.
- **Telemetry:** network rates leave out loopback, which carries the robot's own DDS traffic, so they show the robot's real network use. Expect much lower numbers than before. Disk and network rates are sent to 0.1 KiB/s.
- **Configuration from the app** is applied all or nothing. A push with a malformed field changes nothing, apart from turning motion commands off, and a malformed per-topic limit can no longer stop the agent.
- `initial_state` names the agent's own node, so a robot's Nav2 can be told apart when several robots share a ROS domain.

### Captures

- A bundle capture of a topic that isn't an image (such as `std_msgs/String`) no longer stops the agent. That capture fails with `encode_failed`, and the rest of the bundle is still sent.
- A wall fit on a topic that isn't a `LaserScan` answers `not_a_laserscan`.
- A wall fit gives the LiDAR's mount transform a moment to arrive, instead of sometimes falling back to assuming it faces forward.

### Logs

- On Humble, the agent starts Humble's own `graph_watcher` first and no longer prints why the other builds failed.
- A line of `graph_watcher` output that isn't UTF-8 no longer stops graph updates.
- A robot without Nav2's docking messages logs once, at info, that dock status is off, instead of a warning at every start.

### Documentation

- The README starts the agent with `osiris` and lists `OSIRIS_WS_URL` for a self-hosted Osiris.
- It describes the behavior tree settings as the ROS 2 parameters the agent reads (`bt_mode`, `bt_host`, `bt_server_port`, `bt_publisher_port`). The `OSIRIS_BT_*` environment variables it used to describe were never read.
- The README has new sections on robot commands and bags, and the install line for Ubuntu 24.04 and newer.
- New: `SECURITY.md` (report vulnerabilities privately) and `CONTRIBUTING.md`.
- The package is described as for ROS 2 Humble and later. Its dependencies are unchanged.
