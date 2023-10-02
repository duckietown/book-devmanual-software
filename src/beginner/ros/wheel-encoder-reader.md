(ros-sub-wheel-encoders)=
# Subscribe to wheel encoders

```{needget}
* A Duckietown robot turned ON and visible on `dts fleet discover`
---
* Learn how to receive wheel encoder data from your robot using a **ROS Subscriber**
```

## Topic and message type of interest

For the wheel encoders, the topics used by the Duckiebot to publish the encoder ticks are

* `/ROBOT_NAME/left_wheel_encoder_node/tick`
* `/ROBOT_NAME/right_wheel_encoder_node/tick`

And the message type used over these topics is **duckietown_msgs/WheelEncoderStamped** and contains the following fields.

```
uint8 ENCODER_TYPE_ABSOLUTE=0
uint8 ENCODER_TYPE_INCREMENTAL=1
std_msgs/Header header
int32 data
uint16 resolution
uint8 type
```

where,
- `header`: is the [standard ROS header](https://wiki.ros.org/msg#Header) object;
- `data`: is the current accumulated number of ticks on that motor;
- `resolution`: is how many ticks will be recorded when the motor spins for a full revolution (360 degrees);
- `type`: indicates the type of the encoder, `absolute` or `incremental`, and it takes the values from the constants `ENCODER_TYPE_ABSOLUTE` and `ENCODER_TYPE_INCREMENTAL` defined in the message itself. For a detailed explanation of the difference between the two types of encoders, we direct the reader to [this page](https://en.wikipedia.org/wiki/Rotary_encoder#Basic_types);

For example, on the *DB21* series robot, the resolution is `135` and the type is `1` (`ENCODER_TYPE_INCREMENTAL`). This means that each motor records `135` ticks per full revolution, and that `data=0` at whatever the initial position of the wheel was when the robot was turned ON.

```{note}
If the wheels are spun by hand, the ticks only increase. The robot can only sense direction (hence decrease the counter) when the wheels are spun by the motors.
```

(ros-wheel-encoder-reader-node-create)=
## Create Subscriber ROS Node

We now use our favorite text editor to create the file 
`wheel_encoder_reader_node.py` inside the `src/` directory of our catkin package and add the following content,

```python
#!/usr/bin/env python3

import os
import rospy
from duckietown.dtros import DTROS, NodeType
from duckietown_msgs.msg import WheelEncoderStamped


class WheelEncoderReaderNode(DTROS):

    def __init__(self, node_name):
        # initialize the DTROS parent class
        super(WheelEncoderReaderNode, self).__init__(node_name=node_name, node_type=NodeType.PERCEPTION)
        # static parameters
        self._vehicle_name = os.environ['VEHICLE_NAME']
        self._left_encoder_topic = f"/{self._vehicle_name}/left_wheel_encoder_node/tick"
        self._right_encoder_topic = f"/{self._vehicle_name}/right_wheel_encoder_node/tick"
        # temporary data storage
        self._ticks_left = None
        self._ticks_right = None
        # construct subscriber
        self.sub_left = rospy.Subscriber(self._left_encoder_topic, WheelEncoderStamped, self.callback_left)
        self.sub_right = rospy.Subscriber(self._right_encoder_topic, WheelEncoderStamped, self.callback_right)

    def callback_left(self, data):
        # log general information once at the beginning
        rospy.loginfo_once(f"Left encoder resolution: {data.resolution}")
        rospy.loginfo_once(f"Left encoder type: {data.type}")
        # store data value
        self._ticks_left = data.data

    def callback_right(self, data):
        # log general information once at the beginning
        rospy.loginfo_once(f"Right encoder resolution: {data.resolution}")
        rospy.loginfo_once(f"Right encoder type: {data.type}")
        # store data value
        self._ticks_right = data.data

    def run(self):
        # publish received tick messages every 0.05 second (20 Hz)
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            if self._ticks_right is not None and self._ticks_left is not None:
                # start printing values when received from both encoders
                msg = f"Wheel encoder ticks [LEFT, RIGHT]: {self._ticks_left}, {self._ticks_right}"
                rospy.loginfo(msg)
            rate.sleep()

if __name__ == '__main__':
    # create the node
    node = WheelEncoderReaderNode(node_name='wheel_encoder_reader_node')
    # run the timer in node
    node.run()
    # keep spinning
    rospy.spin()

```

Again, we make our node executable,

    chmod +x ./packages/my_package/src/wheel_encoder_reader_node.py



## Define launcher

Similarly to what we did in the section [](ros-sub-define-launcher), we create a new launcher file
`./launchers/wheel-encoder-reader.sh` with the content,

```shell
#!/bin/bash

source /environment.sh

# initialize launch file
dt-launchfile-init

# launch subscriber
rosrun my_package wheel_encoder_reader_node.py

# wait for app to end
dt-launchfile-join
```

Let us now re-compile our project using the command

    dts devel build -f



## Launch the node

We are now ready to run our reader node,

    dts devel run -R ROBOT_NAME -L wheel-encoder-reader

The `resolution` and `type` values will be printed at the top of the logs once.

Then in the console, the received left and right encoder data will be printed. Try:

* spinning the left/right wheel, in both directions
* driving the robot back and forth with the [virtual joystick](book-opmanual-duckiebot:rc-control)

Observe how the values change in both cases.

If you want to stop the node, just use `Ctrl+C` in the terminal.

```{admonition} Congratulations 🎉
You just built and run a ROS node capable of reading information from the wheel encoder sensors on the Duckiebot.
```
