# bagtube
A simple bag/live topic player for ROS2.
It is intended to be used for remote monitoring of robot operation.

## Server
The basic mechanism of bagtube is republishing of live/bag topics to new topics on demand,
so that a remote communication protocol (e.g. Zenoh) can be configured to stream only the new topics,
sparing the bandwidth during periods when robot activity is of no interest for the user.

### Features
- Livestreaming of ROS2 topics on demand (i.e. toggling republish on/off) via `enable_livestream` service
- Recording bags via `record_bag` action
- Calling additional `SetBool` services on start/stop of recording/livestreaming (specified via `toggle_input_nodes_running_services` parameter,
see [example config](params/chatter_camera.yaml))
- Listing of recorded bags via `get_bag_list` service
- Limiting of total used storage via `max_total_size_gb` parameter
- Playback of bags via `play_bag` action
- Playback control via `control_playback` service (play/pause/seek)
- Sending of preview snapshots after each seek (optional if `snapshot_bag_stamp` is specified in `control_playback` service)

## Client
The client is not implemented yet, but it will be a simple RViz2 panel for controlling the server.

## Example

Installation:
```bash
sudo apt install ros-iron-ros-base ros-iron-rviz2 ros-iron-demo-nodes-cpp ros-iron-usb-cam
cd ~/my_ws/
. /opt/ros/iron/setup.bash
colcon build --symlink-install
```
Running server:
```bash
. ~/my_ws/install/setup.bash
ros2 launch bagtube chatter_camera.launch.py
```
Displaying the stream:
```bash
. /opt/ros/iron/setup.bash
ros2 run rviz2 rviz2
# add Image display and set topic to /image_stream
```
```bash
. /opt/ros/iron/setup.bash
ros2 topic echo /chatter_stream
```
Commands:
```bash
. ~/my_ws/install/setup.bash
# livestream
ros2 service call /enable_livestream std_srvs/srv/SetBool "data: true"
ros2 service call /enable_livestream std_srvs/srv/SetBool "data: false"

# record
ros2 action send_goal /record_bag bagtube_msgs/action/RecordBag "name: 'test'
max_duration: 0.0" -f
[Ctrl+C]

# play a listed bag
ros2 service call /get_bag_list bagtube_msgs/srv/GetBagList "name: ''"
# use the stamp from the list in the next command
ros2 action send_goal /play_bag bagtube_msgs/action/PlayBag "name: 'test'
stamp:
  sec: 1693574434
  nanosec: 992896000
start_offset: 0.0" -f
```
```bash
. ~/my_ws/install/setup.bash
#pause (offset, snapshot bag name and stamp are ignored)
ros2 service call /control_playback bagtube_msgs/srv/ControlPlayback "command: 0
offset: 0.0
snapshot_bag_name: ''
snapshot_bag_stamp:
  sec: 0
  nanosec: 0"

#resume (offset, snapshot bag name and stamp are ignored)
ros2 service call /control_playback bagtube_msgs/srv/ControlPlayback "command: 1
offset: 0.0
snapshot_bag_name: ''
snapshot_bag_stamp:
  sec: 0
  nanosec: 0"

#seek
ros2 service call /control_playback bagtube_msgs/srv/ControlPlayback "command: 2
offset: 3.0
snapshot_bag_name: 'test'
snapshot_bag_stamp:
  sec: 1693574434
  nanosec: 992896000"
```