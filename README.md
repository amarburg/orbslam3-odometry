# orbslam3-odometry

## Introduction
This repository is ROS2 wrapper around ORBSLAM3.  It publishes ORBSLAM3 Odometry on a ROS2 topic.

`orbslam3_odometry.launch.py`, `orblsam3_odometry.yaml`, and `aquarium_nano_stereo.yaml` have all been "tuned" for our cameras and data.

## Prerequisites


I was testing on the desktop under ROS Jazzy.  I found the system worked much better with [Zenoh as the middleware](https://github.com/ros2/rmw_zenoh)

Jetpack / Nano build it tbd.

## To build

(These instructions have been tested on a 24.04 desktop, not on the Nano)

First, install [ROS2](https://docs.ros.org/en/jazzy/Installation.html).

1. Install [ORBSLAM3](https://github.com/UZ-SLAMLab/ORB_SLAM3) per their [build instructions](https://github.com/UZ-SLAMLab/ORB_SLAM3#3-building-orb-slam3-library-and-examples).  Note that (for now) ORBSLAM does not need to be in a ROS workspace -- you do need to manually specify the path to ORBSLAM in the ROS build(below), that's something that can be fixed in the future.

2. Create a ROS2 workspace and source ROS

```bash
mkdir -p orbslam_ws/src/ && cd orbslam_ws/src/
source /opt/ros/$ROS_VERSION/setup.bash
```

2. Clone this repository into the workspace
``` bash
git clone https://github.com/Il-castor/orbslam3-odometry.git
```

2. Change this [line](src/orbslam3_odometry/cmake/FindORB_SLAM3.cmake#L8) to your own `ORB_SLAM3` path from step 1.

Install dependencies:

```bash
cd ..
rosdep install --ignore-src --from-paths src
```

Then build
``` bash
colcon build --symlink-install
```

## Troubleshootings
If you cannot find `sophus/se3.hpp`:
Go to your `ORB_SLAM3_ROOT_DIR` and install sophus library.

``` bash
$ cd ~/{ORB_SLAM3_ROOT_DIR}/Thirdparty/Sophus/build
$ sudo make install
```

## How to use on Aquarium data

The current `orbslam3_odometry.launch.py` is specialized for running the bagfiles from the 4/25/25 Aquarium testing.

orbslam is configured to take _rectified_ images. In addition to `orbslam3_odometry` is runs two `image_proc::RectifyNode`s to rectify the left and right images.

 If I read [this line](https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4/src/Settings.cc#L337) correctly, with Rectified images, `Stereo.T_c1_c` does not need to be set (it is effectively redundant with the stereo baseline) but not providing it didn't work TBD.


First, source the workspace
``` bash
$ source install/setup.bash
```

### Publishing supplemental camera info

The bagfiles captured on 4/25/25 at the Aquarium do not contain `camera_info` topics.   The local script [camera_info_injector](orbslam_helpers/orbslam_helpers/camera_info_injector.py) can address this by listens to an image stream, then republishes that image along with a time-synchronized `camera_info` topic.   For stereo data, two instances can be launched with [`inject_camera_info.launch.py`](orbslam3_odometry/launch/inject_camera_info.py).

The injector reads the left and right calibration from this repo.

**NOTE:**  This launchfile assumes the _input_ image topics are in the `/bag/` namespace;  it then publishes its output in the standard locations of `/left/image_raw`, `/left/camera_info` etc.

To run the injector:

```
(in on terminal)     > ros2 launch orbslam_odometry inject_camera_info.launch.py
```


3. Launch
``` bash
(in a second terminal) ros2 launch orbslam3_odometry stereo.launch.py
```


To stop the node press `ctrl-c` and this save ORB_SLAM3 statistics txt in your folder.

4. Play back a bagfile while remapping the topics:

```
(in a third terminal) >  ros2 bag play  -m /left/image_raw:=/bag/left/image_raw /right/image_raw:=/bag/right/image_raw -i <bagfile>
```
