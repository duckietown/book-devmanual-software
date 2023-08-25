(ros-pub-node)=
# ROS Publisher

```{needget}
* A Duckietown robot turned ON and visible on `dts fleet discover`
---
* Learn how to create a new **ROS Node** publishing messages using a **ROS Publisher**
```

The most common communication pattern in Robotics is known as
[`publish-subscribe`](https://en.wikipedia.org/wiki/Publish%E2%80%93subscribe_pattern).
ROS implements the `publish-subscribe` pattern using **ROS Publishers** and **ROS Subscribers**.
In this section, we will learn to create a **ROS Publisher**.

The general concept is simple, a publisher has the job of publishing messages from a ROS node into
the ROS network for other nodes to receive (using **ROS Subscribers**).


(ros-pub-node-create)=
## Create Publisher ROS Node

We will see how to write a simple ROS program with Python, but any language supported by ROS should do it.
In [](ros-catkin-package-create), we learned how to make a new Catkin package, we will now populate that
package with a ROS node hosting a ROS Publisher.

Nodes are placed inside the directory `src/` of a Catkin package.
Let us go ahead and create the directory `src` inside `my_package`.
We can do so by running the following command from the root of our DTProject.

    mkdir -p ./packages/my_package/src

We now use our favorite text editor to create the file 
`my_publisher_node.py` inside the `src/` directory we just created and place the following code in it:

```python
#!/usr/bin/env python3

import os
import rospy
from std_msgs.msg import String
from duckietown.dtros import DTROS, NodeType


class MyPublisherNode(DTROS):

    def __init__(self, node_name):
        # initialize the DTROS parent class
        super(MyPublisherNode, self).__init__(node_name=node_name, node_type=NodeType.GENERIC)
        # static parameters
        self._vehicle_name = os.environ['VEHICLE_NAME']
        # construct publisher
        self._publisher = rospy.Publisher('chatter', String, queue_size=10)

    def run(self):
        # publish message every 1 second (1 Hz)
        rate = rospy.Rate(1)
        message = f"Hello from {self._vehicle_name}!"
        while not rospy.is_shutdown():
            rospy.loginfo("Publishing message: '%s'" % message)
            self._publisher.publish(message)
            rate.sleep()

if __name__ == '__main__':
    # create the node
    node = MyPublisherNode(node_name='my_publisher_node')
    # run node
    node.run()
    # keep the process from terminating
    rospy.spin()
```

```{note}
Using the super class `DTROS` provided by the Python module `duckietown.dtros` is not mandatory but it
is highly suggested as it provides a lot of useful features that plain ROS does not. More on these later.
```

We now need to the tell our file system that we want our file `my_publisher_node.py` be treated 
as an executable file. We do so by running the following command from the root of our DTProject:

    chmod +x ./packages/my_package/src/my_publisher_node.py



## Define launcher

As we discussed above, everything in Duckietown runs inside Docker containers. This means that we also need 
to tell Docker what to run when the container is started. In this case, we want our new ROS publisher node
to run.

Each DTProject compiles into a single Docker image, but we can declare multiple start "behaviors" for the 
same project/image so that the same project can serve multiple (though related) purposes. As we learned
in [](dtproject-launchers), we can use **launchers** to accomplish this. As we learned in 
[](dtproject-launcher-add-new), we create a new launcher to allow for this new start behavior.

In order to do so, we create the file `./launchers/my-publisher.sh` and add the following content,

```shell
#!/bin/bash

source /environment.sh

# initialize launch file
dt-launchfile-init

# launch publisher
rosrun my_package my_publisher_node.py

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

    dts devel run -H ROBOT_NAME -L my-publisher

This will show the following message:

```
...
==> Launching app...
[INFO] [1693000564.020676]: [/my_publisher_node] Initializing...
[INFO] [1693000564.028260]: [/my_publisher_node] Node starting with switch=True
[INFO] [1693000564.029052]: [/my_publisher_node] Found 0 user configuration files in '/data/config/nodes/generic'
[INFO] [1693000564.029608]: [/my_publisher_node] Found 0 user configuration files in '/data/config/nodes/my_publisher_node'
[INFO] [1693000564.034693]: [/my_publisher_node] Health status changed [STARTING] -> [STARTED]
[INFO] [1693000564.035819]: Publishing message: 'Hello from vbot!'
[INFO] [1693000565.036489]: Publishing message: 'Hello from vbot!'
[INFO] [1693000566.036378]: Publishing message: 'Hello from vbot!'
...
```

```{admonition} Congratulations 🎉
You just built and run your first Duckietown-compliant and Duckiebot-compatible ROS publisher.
```

If you want to stop it, just use `Ctrl+C`.
