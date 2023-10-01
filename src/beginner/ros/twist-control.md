(ros-pub-twist)=
# Publish Twist2D Controls 

```{needget}
* A Duckietown robot turned ON and visible on `dts fleet discover`
---
* Learn how to control the Duckiebot's chasis linear and angular velocities using a **ROS Publisher**
```

## Topic and message type of interest

The topic used by the Duckiebot to receive Twist commands and compute inverse kinematics to obtain lower level
wheel commands is `/ROBOT_NAME/car_cmd_switch_node/cmd`, while the message type used over this topic is
**duckietown_msgs/Twist2DStamped** which contains the following fields.

```
std_msgs/Header header
float32 v
float32 omega
```

where,
- `header`: is the [standard ROS header](https://wiki.ros.org/msg#Header) object;
- `v`: is the linear velocity in `m/s` with positive signs in the forward driving direction
- `omega`: is the angular velocity in `rad/s` with positive signs in counter-clockwise direction when looking down to the Duckiebot

```{note}
The commanded expected `v` and `omega` relies on good kinematic calibration and car model to function.
```

(ros-twist-control-node-create)=
## Create Publisher ROS Node

We now use our favorite text editor to create the file 
`twist_control_node.py` inside the `src/` directory of our catkin package and add the following content,

```python
#!/usr/bin/env python3

import os
import rospy
from duckietown.dtros import DTROS, NodeType
from duckietown_msgs.msg import Twist2DStamped


# Twist command for controlling the linear and angular velocity of the frame
VELOCITY = 0.3  # linear vel    , in m/s    , forward (+)
OMEGA = 4.0     # angular vel   , rad/s     , counter clock wise (+)


class TwistControlNode(DTROS):

    def __init__(self, node_name):
        # initialize the DTROS parent class
        super(TwistControlNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        # static parameters
        vehicle_name = os.environ['VEHICLE_NAME']
        twist_topic = f"/{vehicle_name}/car_cmd_switch_node/cmd"
        # form the message
        self._v = VELOCITY
        self._omega = OMEGA
        # construct publisher
        self._publisher = rospy.Publisher(twist_topic, Twist2DStamped, queue_size=1)

    def run(self):
        # publish 10 messages every second (10 Hz)
        rate = rospy.Rate(10)
        message = Twist2DStamped(v=self._v, omega=self._omega)
        while not rospy.is_shutdown():
            self._publisher.publish(message)
            rate.sleep()

    def on_shutdown(self):
        stop = Twist2DStamped(v=0.0, omega=0.0)
        self._publisher.publish(stop)

if __name__ == '__main__':
    # create the node
    node = TwistControlNode(node_name='twist_control_node')
    # run node
    node.run()
    # keep the process from terminating
    rospy.spin()
```

Again, we make our node executable,

    chmod +x ./packages/my_package/src/twist_control_node.py


## Define launcher

We create a new launcher file `./launchers/twist-control.sh` with the content,

```shell
#!/bin/bash

source /environment.sh

# initialize launch file
dt-launchfile-init

# launch subscriber
rosrun my_package twist_control_node.py

# wait for app to end
dt-launchfile-join
```

Let us now re-compile our project using the command

    dts devel build -f


## Launch the node


```{danger}
The robot's wheels will start moving and spinning as soon as the node is launched. Please, make sure that 
your robot has enough space to drive around without the risk of harming somebody or himself 
(e.g., by falling off a desk).
```

We run the node,

    dts devel run -R ROBOT_NAME -L twist-control

And observe the wheels rotate as instructed.
If you want to stop it, just use `Ctrl+C`, and the wheels should stop spinning as per the behavior 
defined in the function `on_shutdown()` above.

```{admonition} Congratulations 🎉
You just built and run a ROS node capable of interacting with the Duckiebot and control one
of its actuators in another way.
```
