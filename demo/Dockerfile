# Osiris Cloud's demo robot: Gazebo's TurtleBot3 under Nav2 Jazzy, with the
# Osiris bridge from PyPI. start.sh runs it, and fly.toml deploys it.
FROM ros:jazzy-ros-base

# Xvfb and Mesa render the simulated lidar and camera in software.
RUN apt-get update -qq \
 && DEBIAN_FRONTEND=noninteractive apt-get install -y -qq --no-install-recommends \
      ros-jazzy-navigation2 ros-jazzy-nav2-bringup ros-jazzy-nav2-minimal-tb3-sim \
      ros-jazzy-ros-gz-sim ros-jazzy-ros-gz-bridge \
      python3-pip iproute2 procps xvfb libegl1 libegl-mesa0 libgl1-mesa-dri \
 && rm -rf /var/lib/apt/lists/*

# Nav2's lifecycle managers reset the whole stack when a server misses its 4 s
# heartbeat, which a busy machine can, and Nav2 then crashes in the reset. The
# launch files fix the managers' parameters, so the timeout is raised there.
RUN for f in /opt/ros/jazzy/share/nav2_bringup/launch/navigation_launch.py /opt/ros/jazzy/share/nav2_bringup/launch/localization_launch.py; do \
      sed -i \
        -e "s/{'autostart': autostart, 'node_names': lifecycle_nodes}/{'autostart': autostart, 'node_names': lifecycle_nodes, 'bond_timeout': 20.0}/" \
        -e "s/{'node_names': lifecycle_nodes}\]/{'node_names': lifecycle_nodes}, {'bond_timeout': 20.0}]/" "$f" \
      && grep -q bond_timeout "$f"; \
    done

# Nav2's settings with the global costmap published every 5 s instead of every
# second: the Map pane streams it, and each one takes about 150 KB to store.
RUN mkdir -p /demo \
 && sed '/^global_costmap:/,/^[a-z]/ s/publish_frequency: 1.0/publish_frequency: 0.2/' \
      /opt/ros/jazzy/share/nav2_bringup/params/nav2_params.yaml > /demo/nav2_params.yaml \
 && grep -A6 '^global_costmap:' /demo/nav2_params.yaml | grep -q 'publish_frequency: 0.2'

ARG BRIDGE_VERSION=0.5.2
RUN pip3 install --no-cache-dir --break-system-packages osiris-bridge==${BRIDGE_VERSION}

COPY start.sh clean_bags.sh initial_pose.py patrol.py osiris_params.yaml /demo/
CMD ["/demo/start.sh"]
