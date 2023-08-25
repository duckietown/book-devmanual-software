(ros-launch)=
# Launch files

You edited the `launch.sh` file to remove `roscore` when it was already running. What if there was something which
starts a new `rosmaster` when it doesn't exist?

You also added multiple `rosrun` commands to run the publisher and subscriber. Now imagine writing similar shell scripts for programming multiple robot behaviors. Some basic nodes such as a camera or a motor driver will be running in all operation scenarios of your Duckiebot, but other nodes will be added/removed to run specific behaviors (e.g. lane following with or without obstacle avoidance). You can think of this as a hierarchy where certain branches are activated optionally.

You can obviously write a "master" `launch.sh` which executes other shell scripts for hierarchies. How do you pass parameters between these scripts? Where do you store all of them? What if you want to use packages created by other people?

ROS again saves the day by providing us with a tool that handles all this! This tool is called [roslaunch](http://wiki.ros.org/roslaunch).


In this section, you will see how to use a ROS launch file to start both the publisher and subscriber together.

Create a folder called `launch` inside your package and then create a file inside the folder called `multiple_nodes.launch` with the following content

```xml
<launch>

  <node pkg="my_package" type="my_publisher_node.py" name="my_publisher_node" output="screen"/>
  <node pkg="my_package" type="my_subscriber_node.py" name="my_subscriber_node"  output="screen"/>

</launch>
```


Then replace the following lines inside `launch.sh` file

```
rosrun my_package my_node.py &
rosrun my_package my_node_subscriber.py
```

with

    roslaunch my_package multiple_nodes.launch

Build and run the image again like above. You should get the same result.

You can read more about how to interpret launch files [here](http://wiki.ros.org/roslaunch/XML).