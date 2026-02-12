# event_ros

`event_ros` is a ROS2 workspace repository that aggregates event-camera-related packages (driver, messages, codecs, renderer, tools, and bridge packages).

## Repository layout

- `libcaer_driver`: ROS2 driver for iniVation cameras (DAVIS, DVXplorer)
- `event_camera_msgs`: core message definitions
- `event_camera_codecs`: event encoding/decoding library
- `event_camera_renderer`: event-to-image renderer nodes
- `event_camera_tools`: analysis and conversion tools
- `event_camera_legacy_tools`: compatibility tools for legacy message formats
- `event_bridge_cpp`, `event_bridge_py`: bridge packages in this monorepo

## 1. Clone and initialize submodules

```bash
cd ~/ws/src
git clone https://github.com/taehun-ryu/event_ros.git
cd event_ros
git submodule update --init --recursive
```

If the repository is already cloned, run only:

```bash
cd <event_ros_repo>
git submodule update --init --recursive
```

## 2. Install libcaer

```bash
sudo add-apt-repository ppa:ubuntu-toolchain-r/test
sudo add-apt-repository ppa:inivation-ppa/inivation
sudo apt update
sudo apt install libcaer
```

## 3. Install build tools and ROS dependency helpers

```bash
sudo apt update
sudo apt install -y \
  build-essential \
  cmake \
  git \
  python3-colcon-common-extensions \
  python3-rosdep \
  python3-vcstool
```

Initialize `rosdep` once (skip if already initialized):

```bash
sudo rosdep init
rosdep update
```

## 4. Install ROS package dependencies

```bash
export ROS_DISTRO=humble   # or jazzy
source /opt/ros/${ROS_DISTRO}/setup.bash
cd <event_ros_repo>
rosdep install --from-paths . --ignore-src --rosdistro ${ROS_DISTRO} -r -y
```

## 5. Build

```bash
source /opt/ros/${ROS_DISTRO}/setup.bash
cd <event_ros_repo>
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
source install/setup.bash
```

## 6. Troubleshooting

- `Package ... not found` during build:
  confirm submodules are initialized (`git submodule update --init --recursive`).
- `libcaer` not found:
  re-check section 2 and verify installation with `dpkg -l | grep libcaer`.
- Permission errors when opening camera:
  reconnect the device and confirm your user has the required USB access on your system.
- `rosdep` unresolved keys:
  run `rosdep update` and verify that `ROS_DISTRO` is set correctly.
