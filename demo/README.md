# Demo robot

Osiris Cloud's demo robot: a simulated TurtleBot3 in Gazebo with the full Nav2
stack, driving between four points of its map on its own, with the Osiris
bridge connected to the demo workspace. People invited to the demo workspace
watch it live, ask the Agent about it and record bags. Robot commands are off.

- `Dockerfile`: ROS 2 Jazzy, Gazebo Harmonic, Nav2 and the bridge from PyPI
  (`BRIDGE_VERSION`). Nav2 runs with its own settings except the global
  costmap, published every 5 s instead of every second to keep the stored
  data small.
- `start.sh`: starts the simulation and gives AMCL the robot's pose, then runs
  the bridge, the patrol and the recording clean-up. If the simulation, the
  bridge or the patrol stops, the machine starts over.
- `patrol.py`: the Nav2 goals, `initial_pose.py`: the robot's pose for AMCL,
  `clean_bags.sh`: stops a recording after 5 minutes or 1 GB and deletes it
  after an hour, `osiris_params.yaml`: the bridge's fixed settings.
- `fly.toml`: the Fly app `osiris-demo-robot`, one performance-2x machine (2
  CPUs, 4 GB) in ord, connected to the gateway over Fly's private network.

## Turning it on and off

```
./robot.sh on       # in the demo workspace within a minute
./robot.sh off      # stopped: only its disk is billed
./robot.sh status
```

Running, it costs about $0.09 an hour, billed by the second.

## Deploying

From this folder: `fly deploy -c fly.toml --remote-only`. The bridge's token is
the `OSIRIS_AUTH_TOKEN` secret: the token of the demo workspace's robot.
