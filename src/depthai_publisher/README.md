# depthai_publisher
EGB349 OAK-D Lite DepthAI Publisher

## Ensure you have cv-bridge and vision-opencv packages installed using aptitude store and install depthai packages using pip

```
sudo apt-get install ros-noetic-compressed-image-transport
sudo apt-get install ros-noetic-camera-info-manager
sudo apt-get install ros-noetic-rqt-image-view
sudo apt-get install ros-noetic-cv-bridge
sudo apt-get install ros-noetic-vision-opencv

sudo apt-get install python3-pip
python3 -m pip install -U depthai
```

## Step 1 - Clone Repository

```
cd [ros_ws]/src/
git clone https://github.com/Piaktipik/depthai_publisher.git
```

## Step 2 - Build Packages
```
cd ../
```

#### Source ROS Noetic packages

```
catkin_make
```

## Step 3 - Run the following ros nodes

#### Terminal 1:
```
roscore
```

#### Terminal 2 (run oak-d frame publisher):
```
rosrun depthai_publisher dai_publisher
```

#### Terminal 3 (run arucomarker identification):
```
rosrun depthai_publisher aruco_subscriber
```

#### Terminal 4 (visualisation):
```
rqt_image_view
```
OR
```
rviz
```

## GCS voice announcements

The target announcer reports confirmed map-frame YOLO and ArUco observations.
It publishes the same text on `/gcs/target_alert` and generates speech locally
on the GCS through `spd-say`.

```bash
roslaunch depthai_publisher object_announcer.launch
```

The independent mission announcer reports arming changes, mission start,
payload commands, landing-marker selection and successful landing. It publishes
the same text on `/gcs/mission_alert`.

```bash
roslaunch depthai_publisher mission_announcer.launch
```

These nodes are GCS presentation components. They do not command navigation,
payload hardware or the detector, and they are not started by the onboard
mission launch files.
