(ros-cli)=
# CLI - Command Line Interface







// OLD FROM HERE ON



(ros-multi-agent)=
## Multi-agent Communication

In this subsection, you will learn how to communicate between your laptop and the Duckiebot using ROS. Start by
verifying that _Portainer_ is running.

Next, ping your Duckiebot to find its IP address:

    ping MY_ROBOT.local

Note down the address. Next, find the IP address of your computer. Note that you may have multiple IP addresses depending on how many networks you are connected to. If you have a Linux computer, you can find your IP using:

    ifconfig

From the output, extract the IP address of the interface from which you are connected to your Duckiebot. For example, if you and your Duckiebot are both connected through WiFi, find your IP address from the WiFi connection.

Run the following command:

    docker run -it --rm --net host duckietown/dt-ros-commons:ente /bin/bash

Right now, you are inside a ROS-enabled container which is connected to the `rosmaster` running on your laptop. But you want to connect to the `rosmaster` on your duckiebot. To do this, inside the container, run:

    export ROS_MASTER_URI=http://![MY_ROBOT_IP]:11311/
    export ROS_IP=![MY_IP]

Replace `MY_ROBOT_IP` and `MY_IP` with the IP addresses extracted above, in that order. More information about these environment variables [here](http://wiki.ros.org/ROS/EnvironmentVariables).

Now, run:

    rostopic list

You should see topics from your Duckiebot appearing here. Voilà! You have successfully established connection between your laptop and Duckiebot through ROS!

```{tip}
If the `11311` above seems confusing, no need to worry! This is simply the default port number that ROS 
uses for communication. You can change it for any other free port.
```