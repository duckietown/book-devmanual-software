(ros-sub-node)=
# ROS Subscriber

```{needget}
* A Duckietown robot turned ON and visible on `dts fleet discover`
---
* Learn how to create a new **ROS Node** receiving messages using a **ROS Subscriber**
```

The most common communication pattern in Robotics is known as
[`publish-subscribe`](https://en.wikipedia.org/wiki/Publish%E2%80%93subscribe_pattern).
ROS implements the `publish-subscribe` pattern using **ROS Publishers** and **ROS Subscribers**.
In this section, we will learn to create a **ROS Subscriber**.

The general concept is simple: a subscriber has the job of listening for messages about a specific _topic_
that are published by other ROS nodes (using **ROS Publishers**) over a ROS network. 


(ros-pub-node-create)=
## Create Subscriber ROS Node

In [](ros-catkin-package-create), we learned how to make a new Catkin package, we will now populate that
package with a ROS node hosting a ROS Subscriber.

Again, nodes are placed inside the directory `src/` of a Catkin package.
If we followed the tutorial [](ros-pub-node-create) we should already have this directory.

We now use our favorite text editor to create the file
`my_subscriber_node.py` inside the `src/` directory we just created and place the following code in it:

```python
#!/usr/bin/env python3

import rospy
from duckietown.dtros import DTROS, NodeType
from std_msgs.msg import String

class MySubscriberNode(DTROS):

    def __init__(self, node_name):
        # initialize the DTROS parent class
        super(MySubscriberNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        # construct subscriber
        self.sub = rospy.Subscriber('chatter', String, self.callback)

    def callback(self, data):
        rospy.loginfo("I heard '%s'", data.data)

if __name__ == '__main__':
    # create the node
    node = MySubscriberNode(node_name='my_subscriber_node')
    # keep spinning
    rospy.spin()
```

```{note}
Using the super class `DTROS` provided by the Python module `duckietown.dtros` is not mandatory but it
is highly suggested as it provides a lot of useful features that plain ROS does not. More on these later.
```

We now need to the tell our file system that we want our file `my_subscriber_node.py` be treated
as an executable file. We do so by running the following command from the root of our DTProject:

    chmod +x ./packages/my_package/src/my_subscriber_node.py



## Define launcher

We now create a new launcher file `./launchers/my-subscriber.sh` with the following content inside,

```shell
#!/bin/bash

source /environment.sh

# initialize launch file
dt-launchfile-init

# launch subscriber
rosrun my_package my_subscriber_node.py

# wait for app to end
dt-launchfile-join
```


## Launch the Publisher node

This part assumes that you have a Duckiebot up and running with a known hostname, e.g., `ROBOT_NAME`.
Let us make sure that our robot is ready by executing the command,

    ping ROBOT_NAME.local

If you can ping the robot, you are good to go.

Let us now re-compile our project using the command

    dts devel build -H ROBOT_NAME -f

and run it using the newly defined launcher (we use the flag `-L/--launcher` to achieve this):

    dts devel run -H ROBOT_NAME -L my-subscriber

This will show the following messages before hanging,

```
...
==> Launching app...
[INFO] [1693000997.289437]: [/my_subscriber_node] Initializing...
[INFO] [1693000997.296816]: [/my_subscriber_node] Node starting with switch=True
[INFO] [1693000997.297660]: [/my_subscriber_node] Found 0 user configuration files in '/data/config/nodes/generic'
[INFO] [1693000997.298273]: [/my_subscriber_node] Found 0 user configuration files in '/data/config/nodes/my_subscriber_node'
[INFO] [1693000997.303460]: [/my_subscriber_node] Health status changed [STARTING] -> [STARTED]
...
```

This is because the ROS Subscriber is now waiting for messages to come in. Let us open a new terminal at the
root of the project and launch an instance of the publisher we built previously. We can do so by running the 
following command,

    dts devel run -H ROBOT_NAME -L my-publisher -n publisher

```{note}
We need to add the option `-n publisher` to tell `dts` to allow multiple instances of the same project to
run simultaneously.
```

You should notice that messages will start to appear on the subscriber side. The expected output is the
following,

```
...
==> Launching app...
[INFO] [1693000997.289437]: [/my_subscriber_node] Initializing...
[INFO] [1693000997.296816]: [/my_subscriber_node] Node starting with switch=True
[INFO] [1693000997.297660]: [/my_subscriber_node] Found 0 user configuration files in '/data/config/nodes/generic'
[INFO] [1693000997.298273]: [/my_subscriber_node] Found 0 user configuration files in '/data/config/nodes/my_subscriber_node'
[INFO] [1693000997.303460]: [/my_subscriber_node] Health status changed [STARTING] -> [STARTED]
[INFO] [1693001092.577549]: I heard 'Hello from ROBOT_NAME!'
[INFO] [1693001093.557725]: I heard 'Hello from ROBOT_NAME!'
...
```


```{admonition} Congratulations 🎉
You just built and run your first Duckietown-compliant and Duckiebot-compatible ROS subscriber.
```

If you want to stop it, just use `Ctrl+C`.
