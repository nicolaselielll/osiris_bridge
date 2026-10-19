# Osiris bridge

![PyPI](https://img.shields.io/pypi/v/osiris-bridge.svg)
![Python](https://img.shields.io/pypi/pyversions/osiris-bridge.svg)
![License](https://img.shields.io/pypi/l/osiris-bridge.svg)
![CI](https://github.com/nicolaselielll/osiris_bridge/actions/workflows/ci.yml/badge.svg)

The Osiris bridge connects a ROS 2 robot to [Osiris](https://osirislabs.io), where you watch it live, record bags, look back through its history and ask an AI what it is doing.

It runs on the robot as one ROS 2 node, `osiris_node`. It connects out to the Osiris gateway over a WebSocket, sends up what the robot reports, and takes requests back down: subscribe to a topic, record a bag, drive or spin. The connection is outbound only, so nothing has to be opened, forwarded or exposed on the robot's network.

It is published on PyPI as `osiris-bridge` and started with the `osiris` command.

## Requirements

- Linux on x86_64 or arm64 (aarch64).
- ROS 2 Humble, Jazzy, Kilted or Lyrical, sourced in the shell that starts the bridge.
- A robot added in Osiris, for its token.

## Install

```bash
pip3 install osiris-bridge
```

On Ubuntu 24.04 and newer (ROS 2 Jazzy and later), pip refuses to install into the system's Python unless told to:

```bash
pip3 install --break-system-packages osiris-bridge
```

### Upgrading from osiris-agent

Up to 0.4.0 the bridge was published as `osiris-agent`. Remove it before installing `osiris-bridge`, since both install the same files:

```bash
pip3 uninstall -y osiris-agent && pip3 install osiris-bridge
```

On Ubuntu 24.04 and newer, add `--break-system-packages` to both commands. Your token, settings and the `osiris` command stay the same.

## Connect a robot

1. In Osiris, add the robot. The **Configure bridge** popup opens with the commands below, filled in for your robot.
2. Copy the two lines from step 2 of the popup into the shell (or service) that will run the bridge:

   ```bash
   export OSIRIS_WS_URL=<your Osiris install's gateway address>
   export OSIRIS_AUTH_TOKEN=<this robot's token>
   ```

3. Start the bridge:

   ```bash
   osiris
   ```

Osiris shows the robot as connected within a few seconds.

The token is shown only once, when the robot is created or its token is rotated. If it's lost, an owner of the workspace rotates it in **Configure bridge** (robot menu > Bridge) and the bridge needs the new one. Treat the token like a password.

A token only works with the Osiris install that issued it. On the hosted Osiris, `OSIRIS_WS_URL` can be left out. On a self-hosted install it's required: without it the bridge connects to the hosted Osiris, which refuses the token.

## What it sends

With no configuration at all:

- the ROS graph: nodes, topics, actions, services, lifecycle states and QoS
- CPU, memory, disk, network and battery telemetry, and the busiest processes
- action goals: their status, feedback and outcome
- node parameters
- Nav2's behavior tree, if Nav2 runs

Topic data is sent only for the topics someone subscribes to from Osiris, within rate and size limits (see [Parameters](#parameters)). Raw images are re-encoded to JPEG and point clouds are subsampled before they leave the robot. Node logs (`/rosout`) are a topic like any other: they are sent once `/rosout` is subscribed.

Off until you turn them on: the TF tree (`tf_tree_enabled`), `ros2_control` controllers and hardware (`ros2_control_enabled`), a custom BT.CPP tree (`bt_mode: btcpp`), and robot commands (`commands_enabled`).

Everything is stamped with the robot's own clock, so keep it in sync (`timedatectl set-ntp true`).

## Configuration

The bridge runs in one of two modes, for every setting at once:

1. **Cloud Config** (`osiris` with no parameters): the bridge waits for the configuration saved in Osiris, in the **Cloud Config** tab of **Configure bridge**, and applies it. Changes saved there apply while the bridge runs. A setting Cloud Config leaves out uses its default.
2. **A parameters file** (`osiris --ros-args --params-file osiris_params.yaml`, or any setting given with `-p`): the bridge applies the file at startup and ignores Cloud Config for the whole run, even for settings the file doesn't mention. Editing the file needs a restart.

Two exceptions:

- `bag_output_dir` is set on the robot only, never from Cloud Config. Setting only it keeps Cloud Config in effect: `osiris --ros-args -p bag_output_dir:=/data/bags`.
- Robot commands always need **Allow robot commands** in Osiris, and Osiris applies its own limits too, whichever mode the bridge runs in. See [Robot commands](#robot-commands).

A parameters file sets any of the parameters below under `osiris_node`:

```yaml
osiris_node:
  ros__parameters:
    tf_tree_enabled: true
    bt_mode: nav2
    battery_topic: /battery_state
    topic_data_rate_hz: 50.0
```

### Parameters

| Parameter | Default | What it does |
|---|---|---|
| `telemetry_enabled` | `true` | CPU, memory, disk, network and battery telemetry. |
| `goals_enabled` | `true` | Action goal status and feedback. |
| `params_enabled` | `true` | Node parameters. |
| `tf_tree_enabled` | `false` | TF tree monitoring, from `/tf` and `/tf_static` (or the ones under the bridge's namespace). |
| `tf_tree_poll_interval` | `0.2` s | How often the TF tree is checked for changes. |
| `ros2_control_enabled` | `false` | `ros2_control` controllers and hardware interfaces. |
| `ros2_control_poll_interval` | `2.0` s | How often they are read. |
| `bt_mode` | `nav2` | Behavior tree source: `nav2`, `btcpp` or `off`. See [Behavior trees](#behavior-trees). |
| `bt_host` | `127.0.0.1` | Where the BT.CPP Groot2 publisher runs (`btcpp` only). |
| `bt_server_port` | `1667` | Groot2 server port (`btcpp` only). |
| `bt_publisher_port` | `1668` | Groot2 publisher port (`btcpp` only). |
| `graph_debounce_interval` | `1.0` s | Wait after a graph change before sending it. `0` sends every change at once. |
| `battery_topic` | `/battery_state` | `sensor_msgs/msg/BatteryState` topic for battery telemetry and the battery check. |
| `battery_check_enabled` | `false` | Refuse robot commands below `battery_min_percent`. |
| `battery_min_percent` | `30.0` | Battery threshold for that check. |
| `dock_check_enabled` | `false` | Refuse robot commands while docked. |
| `dock_status_topic` | `/dock_status` | `opennav_docking_msgs/msg/DockStatus` topic for that check. |
| `commands_enabled` | `false` | Accept drive and spin commands. See [Robot commands](#robot-commands). |
| `command_max_speed_mps` | `0.5` m/s | Fastest drive. |
| `command_max_distance_m` | `5.0` m | Longest drive, either direction. |
| `command_max_angle_deg` | `360.0`° | Largest spin, either direction. |
| `command_max_time_allowance_s` | `120.0` s | Longest time allowance for a drive or spin. |
| `bag_output_dir` | `~/ros2_bags` | Where bags recorded from Osiris are saved. Robot only. |
| `topic_data_rate_hz` | `50.0` Hz | Most messages per second sent for each subscribed topic. `0` is no limit. |
| `topic_data_max_bytes_per_sec` | `0.0` | Most bytes per second for each subscribed topic. `0` is no limit. |
| `global_topic_data_max_bytes_per_sec` | `0.0` | Most bytes per second for all subscribed topics together. `0` is no limit. |
| `global_topic_data_max_msgs_per_sec` | `0.0` | Most messages per second for all subscribed topics together. `0` is no limit. |
| `image_jpeg_quality` | `70` | JPEG quality (1 to 95) for raw `sensor_msgs/Image` frames. Compressed images are sent as they are. |
| `image_max_dimension` | `640` px | Longest edge of a raw image after resizing. `0` never resizes. |
| `pointcloud_max_points` | `10000` | Most points per `PointCloud2` frame; larger frames are subsampled. `0` never subsamples. Never more than 300000. |
| `pointcloud_keep_intensity` | `false` | Keep each point's intensity. |
| `pointcloud_keep_rgb` | `false` | Keep each point's color. |
| `pointcloud_default_rate_hz` | `1.0` Hz | Rate for point cloud topics without their own. `0` is no limit. |

In Cloud Config, a topic can also get its own rate, size, image and point cloud limits, from its row in the Graph pane. A parameters file sets only the defaults above.

### Environment variables

| Variable | Required | What it does |
|---|---|---|
| `OSIRIS_AUTH_TOKEN` | Yes | The robot's token. The bridge won't start without it. |
| `OSIRIS_WS_URL` | On a self-hosted install | The gateway to connect to, for example `wss://osiris.example.com/gateway`. Defaults to the hosted Osiris. |
| `OSIRIS_ROBOT_MODEL` or `ROBOT_MODEL` | No | The robot model shown in Osiris. Read from `/proc/device-tree/model` when unset. |

## Behavior trees

- **Nav2** (`bt_mode: nav2`, the default) follows Nav2's own `/behavior_tree_log`, with nothing to set up. A Nav2 under a namespace publishes `<namespace>/behavior_tree_log`; the bridge follows the one in its own namespace, else the only one there is.
- **A custom BT.CPP tree** (`bt_mode: btcpp`) needs Groot2 monitoring turned on in your tree's code (`BT::Groot2Publisher`). Set `bt_host`, `bt_server_port` and `bt_publisher_port` if the publisher isn't on this machine's default ports 1667 and 1668. This mode also sends the tree's blackboard.

Only one mode runs at a time.

## Robot commands

The bridge can run Nav2's `drive_on_heading` and `spin` actions when asked from Osiris. This is **off by default**: while it's off, every motion command is refused. Stopping a running command always works.

An owner of the robot's workspace turns commands on with **Allow robot commands** in **Configure bridge**, where the limits are set too. Osiris sends a motion command only when both Osiris and the bridge allow it, and the stricter limits apply. A parameters file can turn commands off or set stricter limits on this robot, but it can't turn them on while they're off in Osiris:

```yaml
osiris_node:
  ros__parameters:
    commands_enabled: true              # still needs Allow robot commands in Osiris
    command_max_speed_mps: 0.3
```

A command is refused before it reaches Nav2 when:

- a value is over a limit, or missing or not a number,
- another goal is already moving the robot, such as a Nav2 navigation,
- `battery_check_enabled` is on and the battery is below `battery_min_percent`, or has no reading from the last 10 seconds,
- `dock_check_enabled` is on and the robot is docked, or has no reading from the last 10 seconds.

The refusal says which.

## Bags

Bags recorded from Osiris are saved on the robot, one folder per recording, in `bag_output_dir` (default `~/ros2_bags`). Set it on the robot:

```bash
osiris --ros-args -p bag_output_dir:=/data/bags
```

Osiris lists and downloads only finished recordings directly in that folder (folders with a `metadata.yaml`). A download skips anything a symlink points to. The user running the bridge needs write access to the folder.

## Several robots on one ROS network

Run each robot's bridge in that robot's namespace, so it follows that robot's Nav2, TF and behavior tree and sends commands to the right one:

```bash
osiris --ros-args -r __ns:=/robot1
```

## Troubleshooting

- **`OSIRIS_AUTH_TOKEN environment variable must be set`**: export the token in the shell or service that runs `osiris`.
- **The robot never connects, on a self-hosted install**: `OSIRIS_WS_URL` isn't set where `osiris` runs, so the bridge went to the hosted Osiris. Copy both lines from **Configure bridge**.
- **`Authentication failed: Invalid token`**: `OSIRIS_WS_URL` points at another install than the one that issued the token, or the token was copied wrong. Otherwise rotate the token.
- **`Authentication failed: This token was replaced by a newer one`**: someone rotated the token. Use the newest one.
- **`Authentication failed: This token's robot or its workspace no longer exists`**: the robot was deleted. Add it again and use its new token.
- **`Authentication failed: Server error`**: the gateway couldn't check the token. Don't rotate it: the bridge keeps retrying and connects once the server is back.
- **`certificate verify failed`, on a self-hosted install**: the robot doesn't trust the install's certificate. Copy the install's root certificate to the robot (on Ubuntu, into `/usr/local/share/ca-certificates/`, then `sudo update-ca-certificates`).
- **`osiris: command not found` after switching from osiris-agent**: `osiris-agent` was removed after `osiris-bridge` was installed, which deletes the bridge's files. Run `pip3 install --force-reinstall osiris-bridge`.
- **`externally-managed-environment` when installing**: use `pip3 install --break-system-packages osiris-bridge`.
- **Connected, but no nodes or topics**: ROS 2 isn't sourced in the shell that runs the bridge, or `ROS_DOMAIN_ID` differs from the robot's.
- **Empty TF tree**: turn on `tf_tree_enabled`. With several robots, run the bridge in its robot's namespace.
- **Empty behavior tree**: see [Behavior trees](#behavior-trees).
- **No battery data**: `battery_topic` doesn't match the robot's topic (under a namespace it's `<namespace>/battery_state`).
- **History at the wrong times**: the robot's clock is off. Turn on time sync.

## Contributing

Bug reports and pull requests are welcome at https://github.com/nicolaselielll/osiris_bridge. See [CONTRIBUTING.md](https://github.com/nicolaselielll/osiris_bridge/blob/main/CONTRIBUTING.md) to report a bug or send a change, and [SECURITY.md](https://github.com/nicolaselielll/osiris_bridge/blob/main/SECURITY.md) to report a security problem privately.

## License

Apache-2.0. See [LICENSE](https://github.com/nicolaselielll/osiris_bridge/blob/main/LICENSE).

## Changelog

See [CHANGELOG.md](https://github.com/nicolaselielll/osiris_bridge/blob/main/CHANGELOG.md) for what changed in each version.
