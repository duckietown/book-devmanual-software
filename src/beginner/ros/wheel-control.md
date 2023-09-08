(ros-pub-wheels)=
# Publish to wheels

```{needget}
* A Duckietown robot turned ON and visible on `dts fleet discover`
---
* Learn how to control the Duckiebot's wheels using a **ROS Publisher**
```

## Topic and message type of interest

The topic used by the Duckiebot to receive wheel commands
is `/ROBOT_NAME/wheels_driver_node/wheels_cmd`, while the message type used over this topic is
**duckietown_msgs/WheelsCmdStamped** which contains the following fields.

```
std_msgs/Header header
float32 vel_left
float32 vel_right
```

where,
- `header`: is the [standard ROS header](https://wiki.ros.org/msg#Header) object;
- `vel_left`: is the signed duty cycle for the *left* wheel (-1.0: full throttle backwards; 0.0: still; 1.0: full throttle forward)
- `vel_right`: is the signed duty cycle for the *right* wheel (-1.0: full throttle backwards; 0.0: still; 1.0: full throttle forward)

```{note}
There is no physical interpretation of these values, as in velocity or angular velocity, because the commands are PWM duty cycles. Therefore, the term throttle (0% - 100%) is used. And even given with the same throttle commands, the physical velocities would vary with different motors and wheels, which could be then measured by sensors like encoders.
```

(ros-wheel-control-node-create)=
## Create Publisher ROS Node

We now use our favorite text editor to create the file 
`wheel_control_node.py` inside the `src/` directory of our catkin package and add the following content,

```python
#!/usr/bin/env python3

import os
import rospy
from duckietown.dtros import DTROS, NodeType
from duckietown_msgs.msg import WheelsCmdStamped


# throttle and direction for each wheel
THROTTLE_LEFT = 0.5        # 50% throttle
DIRECTION_LEFT = 1         # forward
THROTTLE_RIGHT = 0.3       # 30% throttle
DIRECTION_RIGHT = -1       # backward


class WheelControlNode(DTROS):

    def __init__(self, node_name):
        # initialize the DTROS parent class
        super(WheelControlNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        # static parameters
        vehicle_name = os.environ['VEHICLE_NAME']
        wheels_topic = f"/{vehicle_name}/wheels_driver_node/wheels_cmd"
        # form the message
        self._vel_left = THROTTLE_LEFT * DIRECTION_LEFT
        self._vel_right = THROTTLE_RIGHT * DIRECTION_RIGHT
        # construct publisher
        self._publisher = rospy.Publisher(wheels_topic, WheelsCmdStamped, queue_size=1)

    def run(self):
        # publish 10 messages every second (10 Hz)
        rate = rospy.Rate(0.1)
        message = WheelsCmdStamped(vel_left=self._vel_left, vel_right=self._vel_right)
        while not rospy.is_shutdown():
            self._publisher.publish(message)
            rate.sleep()

    def on_shutdown(self):
        stop = WheelsCmdStamped(vel_left=0, vel_right=0)
        self._publisher.publish(stop)

if __name__ == '__main__':
    # create the node
    node = WheelControlNode(node_name='wheel_control_node')
    # run node
    node.run()
    # keep the process from terminating
    rospy.spin()
```

Again, we make our node executable,

    chmod +x ./packages/my_package/src/wheel_control_node.py


## Define launcher

We create a new launcher file `./launchers/wheel-control.sh` with the content,

```shell
#!/bin/bash

source /environment.sh

# initialize launch file
dt-launchfile-init

# launch subscriber
rosrun my_package wheel_control_node.py

# wait for app to end
dt-launchfile-join
```

Let us now re-compile our project using the command

    dts devel build -f


## Launch the node


```{danger}
The robot's wheels will start spinning as soon as the node is launched. Please, make sure that 
your robot has enough space to drive around without the risk of harming somebody or himself 
(e.g., by falling off a desk).
```

We run the node,

    dts devel run -R ROBOT_NAME -L wheel-control

And observe the wheels rotate as instructed.
If you want to stop it, just use `Ctrl+C`, and the wheels should stop spinning as per the behavior 
defined in the function `on_shutdown()` above.

```{admonition} Congratulations 🎉
You just built and run your first ROS node capable of interacting with the Duckiebot and control one
of its actuators.
```
