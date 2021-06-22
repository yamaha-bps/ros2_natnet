# ```ros2_natnet``` Unofficial NatNet ROS2 Driver for Optitrack systems


[![foxy](https://github.com/yamaha-bps/ros2_natnet/actions/workflows/foxy.yaml/badge.svg)](https://github.com/yamaha-bps/ros2_natnet/actions/workflows/foxy.yaml) [![galactic](https://github.com/yamaha-bps/ros2_natnet/actions/workflows/galactic.yaml/badge.svg)](https://github.com/yamaha-bps/ros2_natnet/actions/workflows/galactic.yaml)

ROS2 interface using the proprietary [NatNet SDK](https://www.optitrack.com/software/natnet-sdk/) from Optitrack which is downloaded at build time.

Currently only handles rigid bodies and markers.

## Installation
Clone into a ```colcon``` workspace and build as usual.

```bash
git clone https://github.com/yamaha-bps/ros2_natnet.git src/natnet
colcon build
```

## Node

### ```natnet_node```

#### Publishes to

 - ```optitrack```  - type ```geometry_msgs/msg/TransformStamped```

#### Parameters

 - ```local_address``` - type ```string```, default ```"127.0.0.1"```

 Address of network interface on client machine.

 - ```server_address``` - type ```string```, default ```"192.168.0.1"```

 Address of computer running the Motive software.

 - ```offest_file``` - type ```string```, default ```""```

 Optional file with offsets P_RB0_RB for the rigid bodies such that the published transform
 P_W_RB = P_W_RB0 * P_RB0_RB where RB0 is the frame defined in the Motive software.

```yaml
- name: RigidBody1
  x: 0.
  y: 0.5
  z: 0.
  qw: 1.
  qx: 0.
  qy: 0.
  qz: 0.
- name: RigidBody2
  x: 0.
  y: 0.
  z: 0.
  qw: 0.
  qx: 0.
  qy: 1.
  qz: 0.
```
