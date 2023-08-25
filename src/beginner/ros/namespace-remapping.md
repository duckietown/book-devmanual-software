(ros-namespace-remap)=
## Namespaces and Remapping

If you went through the above link on launch files, you might have come across the terms namespaces and remapping. Understanding namespaces and remapping is very crucial to working with large ROS software stacks.


Consider you have two Duckiebots - `donald` and `daisy`. You want them to communicate with each other so that you use
one `rosmaster` for both the robots. You have two copies of the same node running on each of them which grabs images from the camera and publishes them on a topic called `/image`. Do you see a problem here? Would it not be better if they were called `/donald/image` and `/daisy/image`? Here `donald` and `daisy` are ROS namespaces.


What if you were dealing with a robot which has two cameras? The names `/daisy/camera_left/image` and `/daisy/camera_right/image` are definitely the way to go. You should also be able to do this without writing a new Python file for the second camera.

Let's see how we can do this. First of all, we need to make sure that all the topics used by your Duckiebot are within its namespace.

Edit the `./packages/my_package/launch/multiple_nodes.launch` to look like this:

```xml
<launch>

  <group ns="$(arg veh)">  

    <node pkg="my_package" type="my_publisher_node.py" name="my_publisher_node" output="screen"/>
    <node pkg="my_package" type="my_subscriber_node.py" name="my_subscriber_node"  output="screen"/>

  </group>

</launch>
```

Then edit the roslaunch command in `./launch.sh` as follows:

    roslaunch my_package multiple_nodes.launch veh:=$VEHICLE_NAME

Build and run the image. Once again run `rqt_graph` like above. What changed?

As a next step, we need to ensure that we can launch multiple instances of the same node with different names, and publishing topics corresponding to those names. For example, running two camera nodes with names `camera_left` and `camera_right` respectively, publishing topics `/my_robot/camera_left/image` and `/my_robot/camera_right/image`.

Notice how the `node` tag in the launch file has a `name` attribute. You can have multiple `node` tags with different names for the same python node file. The name provided here will override the name you give inside the python file for the node.

Edit the `./packages/my_package/launch/multiple_nodes.launch` file to have two publishers and two subscribers as below:

```xml
<launch>

  <group ns="$(arg veh)">  

    <node pkg="my_package" type="my_publisher_node.py" name="my_publisher_node_1" output="screen"/>
    <node pkg="my_package" type="my_publisher_node.py" name="my_publisher_node_2" output="screen"/>
    <node pkg="my_package" type="my_subscriber_node.py" name="my_subscriber_node_1"  output="screen"/>
    <node pkg="my_package" type="my_subscriber_node.py" name="my_subscriber_node_2"  output="screen"/>

   </group>

</launch>
```

Check `rqt_graph`. All communications are happening on one topic. You still cannot differentiate between topics being published by multiple nodes. Turns out doing that is very simple. Open the file `./packages/my_package/src/my_publisher_node.py` and edit the declaration of the publisher from

```python
...
        self.pub = rospy.Publisher('chatter', String, queue_size=10)
...
```

to

```python
...
        self.pub = rospy.Publisher('~chatter', String, queue_size=10)
...
```

All we did was add a tilde(`~`) sign in the beginning of the topic. Names that start with a `~` in ROS are private names. They convert the node's name into a namespace. Note that since the nodes are already being launched inside the namespace of the robot, the node's namespace would be nested inside it. Read more about private namespaces [here](http://wiki.ros.org/Names)

Do this for the subscriber node as well. Run the experiment and observe `rqt_graph` again. This time, switch the
graph type from `Nodes only` to `Nodes/Topics (all)` and uncheck `Hide: Dead sinks` and `Hide: Leaf topics`. Play with these two "Hide" options to see what they mean.

All looks very well organized, except that no nodes are speaking to any other node. This is where the magic of remapping begins.

Edit the `./packages/my_package/launch/multiple_nodes.launch` file to contain the following:

```xml
<launch>

  <group ns="$(arg veh)">  

    <node pkg="my_package" type="my_publisher_node.py" name="my_publisher_node_1" output="screen"/>
    <node pkg="my_package" type="my_publisher_node.py" name="my_publisher_node_2" output="screen"/>

    <node pkg="my_package" type="my_subscriber_node.py" name="my_subscriber_node_1"  output="screen">
        <remap from="~/chatter" to="/$(arg veh)/my_publisher_node_1/chatter"/>
    </node>

    <node pkg="my_package" type="my_subscriber_node.py" name="my_subscriber_node_2"  output="screen">
        <remap from="~/chatter" to="/$(arg veh)/my_publisher_node_2/chatter"/>
    </node>

   </group>

</launch>
```

Check `rqt_graph`. Does it make sense?

Now, replace

```xml
<node pkg="my_package" type="my_subscriber_node.py" name="my_subscriber_node_1"  output="screen">
    <remap from="~/chatter" to="/$(arg veh)/my_publisher_node_1/chatter"/>
</node>
```

with

```xml
<node pkg="my_package" type="my_subscriber_node.py" name="my_subscriber_node_1"  output="screen">
    <remap from="~/chatter" to="my_publisher_node_1/chatter"/>
</node>
```

Does it still work? Why?

How about if you replace it with this:

```xml
<node pkg="my_package" type="my_subscriber_node.py" name="my_subscriber_node_1"  output="screen">
    <remap from="~/chatter" to="/my_publisher_node_1/chatter"/>
</node>
```

How about this?

```xml
<remap from="my_subscriber_node_1/chatter" to="my_publisher_node_1/chatter"/>
<node pkg="my_package" type="my_subscriber_node.py" name="my_subscriber_node_1"  output="screen"/>
```

Or this?

```xml
<remap from="~my_subscriber_node_1/chatter" to="~my_publisher_node_1/chatter"/>
<node pkg="my_package" type="my_subscriber_node.py" name="my_subscriber_node_1"  output="screen"/>
```

Can you explain why some of them worked, while some did not?
