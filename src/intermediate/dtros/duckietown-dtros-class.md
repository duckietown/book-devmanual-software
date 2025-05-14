```{seo}
:description: Learn how to structure a Duckietown ROS node using the DTROS base‑class—covering file layout, parameters, publishers, subscribers, timing utilities, config and launch files.
:keywords: Duckietown, DTROS, ROS node, Python, parameters, publishers, subscribers, config files, launch files
```

(sec:intermediate-dtros)=
# The **DTROS** class

```{needget}
* Familiarity with basic ROS concepts and `rospy`
* Completion of the Docker and ROS beginner modules in this book
---
* Ability to create a well‑structured, documented Duckietown ROS node that inherits from `DTROS`
```

This chapter explains **how to write and organize the code of a ROS node** in Duckietown.  

Correct structure is mandatory: it improves readability, performance, and integration with the rest of the stack.  

Inline documentation goes hand‑in‑hand with code structure; detailed guidance is provided in [](sec:dt_way_code_docs).

---

## 1 · General file layout

* Each ROS node lives in the package’s `src/` directory.  
* File name convention: if the node is called `some_name`, the file **must** be `some_name_node.py`.  
* The file must be executable (`chmod +x`).  
* Implement the logic inside a **single class** named `SomeNameNode`, which **inherits from `DTROS`**.

Skeleton example of `some_name_node.py` structure:

```python
#!/usr/bin/env python3

# ── import external libraries ──────────────────────────────────────────────────────────────
import rospy

# ── package‑internal libs ──────────────────────────────────────────────────────
import library

# ── Duckietown helpers (from DTROS-related classes)────────────────────────────────────────────────────────
from duckietown.dtros import (
    DTROS, NodeType, TopicType,
    DTReminder, DTParam, ParamType
)

# ── import ROS messages and services ───────────────────────────────────────────────────────────
from std_msgs.msg import Float32
from duckietown_msgs.msg import SegmentList, Segment, BoolStamped


class SomeNameNode(DTROS):
    def __init__(self, node_name):
        # class implementation
        ...

if __name__ == '__main__':
    some_name_node = SomeNameNode(node_name='some_name_node')
    rospy.spin()
```

```{warning}
Observe that all nodes in Duckietown should inherit from the super class `DTROS`.
This is a hard requirement.
```

**Why inherit from `DTROS`?** 

The base‑class adds runtime utilities (parameter hot‑reload, timing, debug topics, etc.) and enforces best‑practice defaults making writing and debugging code easier. Common bad practices that should be avoided are:

> Never use wildcard imports (`from pkg import *`).  
> Import exactly what needed, and keep aliases only for established standards (`np`, `plt`, …).

---

## 2 · Initialization: the typical `__init__` structure

There are many details regarding the initialization of the node. This is a breakdown of a sample `__init__` structure for the node.

```python
class SomeNameNode(DTROS):
    def __init__(self, node_name):
        super(SomeNameNode, self).__init__(
            node_name=node_name,
            node_type=NodeType.PERCEPTION
        )

        # ── Parameters ────────────────────────────────────────────────────────
        self.detection_freq = DTParam(
            '~detection_freq',                 # private parameter
            param_type=ParamType.INT,
            min_value=-1,
            max_value=30
        )
        # …

        # ── Generic attributes ───────────────────────────────────────────────
        self.something_happened = None
        self.arbitrary_counter = 0
        # …

        # ── Subscribers ──────────────────────────────────────────────────────
        self.sub_img = rospy.Subscriber(
            'image_rect', Image, self.cb_img
        )
        self.sub_cinfo = rospy.Subscriber(
            'camera_info', CameraInfo, self.cb_cinfo
        )
        # …

        # ── Publishers ───────────────────────────────────────────────────────
        self.pub_img = rospy.Publisher(
            'tag_detections_image/compressed',
            CompressedImage,
            queue_size=1,
            dt_topic_type=TopicType.VISUALIZATION
        )
        self.pub_tag = rospy.Publisher(
            'tag_detections',
            AprilTagDetectionArray,
            queue_size=1,
            dt_topic_type=TopicType.PERCEPTION
        )
```

### 2.1 Node creation

A typical ROS node is initialized with `rospy.init_node(...)`. DTROS does that acting as super-constructor, given the node name.

`DTROS` wraps `rospy.init_node()` and supports **node categorization** for graph visualization. This is useful to visualize the ROS
network as a graph, where nodes represent ROS nodes and edges represent ROS topics. 

In such a graph, it might be convenient to group all the nodes working on the `PERCEPTION` problem together, e.g., to clear the clutter and make the graph easier to read.

The parameter `node_type` in the super constructor allows for this. Use the values from the `NodeType` enumeration. Possible node types are the following:


```python
NodeType.{GENERIC, DRIVER, PERCEPTION, CONTROL, PLANNING, LOCALIZATION,
         MAPPING, SWARM, BEHAVIOR, VISUALIZATION, INFRASTRUCTURE,
         COMMUNICATION, DIAGNOSTICS, DEBUG}
```

### 2.2 Parameters

* Always use private names (`~param_name`), i.e., names relative to the namespace of the node.  
* All parameters should be in the scope of the instance, not the method,
so they should always be declared inside the constructor and start with `self.  `.
* **Do not set default values in code**, but keep them in a YAML configuration file. This avoids potential ambiguities.
* * Declare with `DTParam`; avoid `rospy.get_param()` polling.  


In classic ROS, you get the value of a parameter with `rospy.get_param(...)`.
One of the issues of the ROS implementation of parameters is that a node cannot request
to be notified when a parameter's value changes at runtime. Common solutions to this
problem employ a polling strategy (which consists of querying the parameter server for
changes in value at regular intervals). This is highly inefficient and does not scale.
The `dtros` library provides a solution to this.
Alternatively to using `rospy.get_param(...)`, to return the current value of
a parameter, create a `DTParam` object that automatically updates when a new value is
set. Use `self.my_param = DTParam("~my_param")` to create a `DTParam` object and
`self.my_param.value` to read its value.

### 2.3 Attribute order

Define internal non-ROS attributes **before** creating subscribers, so callbacks never access undefined members.

Here is an example that might fail:


```python
class CoolNode(DTROS):
  def __init__(...):
      self.sub_a = rospy.Subscriber(..., callback=cb_sth, ...)
      self.important_variable = 3.1415


  def cb_sth(self):
      self.important_variable *= 1.0
```


And something that is better:


```python
class CoolNode(DTROS):
  def __init__(...):
      self.important_variable = 3.1415
      sub_a = rospy.Subscriber(..., callback=cb_sth, ...)


  def cb_sth(self):
      self.important_variable *= 1.0
```


### 2.4 Publishers / Subscribers

Finally, initialize all the Subscribers and Publishers as shown above.
The `dtros` library automatically decorates the methods `rospy.Publisher` and `rospy.Subscriber`.
By doing so, new parameters are added. All the parameters added by `dtros` have the prefix
`dt_` (e.g., `dt_topic_type`).
Use the values from the `TopicType` enumeration. Possible types list is identical to the
node types list above.

```{note}
Only declare a topic type in a `rospy.Publisher` call.
```

---

## 3 · Naming conventions

* **snake_case** for variables, functions, methods.  
* **CamelCase** only for class names.  
* Prefixes: `sub_…`, `pub_…`, `cb_…` for subscribers, publishers, callbacks.
* Initalizing publishers and subscribers should always be in the scope of the instance, hence starting with `self.`.

---

## 4 · Graceful shutdown of nodes

Implement `def on_shutdown(self):` when the node must release resources, stop motors, or close files. This is particularly useful if running threads in the background, or there are some files that need to be closed, resources released, or to put the robot into a safe state (e.g., stop motors).

---

## 5 · Debug topics

Debug information helps analyze the behavior and performance of nodes. To avoid the diagnostics interfering with the measured process, it is wise to publish heavy debug data only when someone listens.

A frequent (__but bad design__) way of handling that is to have a topic, to which one can publish a message, which when received will induce the node to start building a publishing the debug message. A much better way, and the one that __should be used__ in Duckietown is to create and publish the debug message _only if_ someone has subscribed to the debug topic.
This functionality is inherited by using `dtros`. Publishers created within a DTROS node export the utility function `anybody_listening()`.

```python
if self.pub_debug_img.anybody_listening():
   debug_img = self.very_expensive_function()
   debug_image_msg = self.bridge.cv2_to_compressed_imgmsg(debug_img)
   self.pub_debug_img.publish(debug_image_msg)
```


Note also that all debug topics should be in the `debug` namespace of the node, e.g., `~debug/debug_topic_name`.


Similarly, a Subscriber created within a DTROS node exports the utility function `anybody_publishing()` that checks whether there are nodes that are currently publishing messages.


---

## 6 · Timing helpers

If you have operations that might take non-trivial amount of computational time,
you can profile them in order to be able to analyze the performance of your node.
`DTROS` has a special context for that which uses the same mechanism as the debug topics.
Hence, if you do not subscribe to the topic with the timing information,
there would be no overhead to your performance.
Therefore, be generous with the use of timed sections.

```python
with self.time_phase("Step 1"):
    run_step_1()

with self.time_phase("Step 2"):
    run_step_2()
```
Then, subscribers to `~debug/phase_times` will see, for each separate section, information about the execution frequency, average computational time,
and also the exact lines of code and the file in which this section appears.

---

## 7 · Config files

If nodes have at least one parameter, then it should have a configuration file.
If there is a single configuration (as is the case with most nodes) this file should
be called `default.yaml`.
Assuming that our node is called `some_node`, the configuration files for the node
should be in the `config/some_node/` directory.


Every parameter used in the implementation of the node should have a default value
in the configuration file.
Furthermore, there should be no default values in the code.
The only place where they should be defined is the configuration file.


---

## 8 · Launch files

Assuming a `some_node` named node, in the `launch` directory of the package there should be an atomic launch file with the name `some_node.launch`, which
launches the node in the correct namespace and loads its configuration parameters.

The launch file content of most nodes will be similar to this minimal atomic launcher (`launch/some_node.launch`):

```xml
<launch>
  <arg name="veh"/>
  <arg name="pkg_name" value="some_package"/>
  <arg name="node_name" default="some_node"/>
  <arg name="param_file_name" default="default" doc="Specify a param file"/>

  <group ns="$(arg veh)">
    <node name="$(arg node_name)"
          pkg="$(arg pkg_name)"
          type="$(arg node_name).py"
          output="screen">
      <rosparam command="load"
                file="$(find some_package)/config/$(arg node_name)/$(arg param_file_name).yaml"/>
    </node>
  </group>
</launch>
```

---

By following these guidelines—and leveraging the capabilities baked into **`DTROS`**, your nodes will be easier to read, debug, and integrate across the Duckietown ecosystem.


<!--
(sec:intermediate-dtros)=
# The **DTROS** class

This section deals with how you should write the code in a ROS node. 
In particular, how to structure it. Writing the code of a node goes hand-in-hand with documenting it, 
but this will be discussed in more detail in [](sec:dt_way_code_docs).

## General structure

All ROS nodes should be in the `src` directory of the respective package.
If the node is called `some_name`, then the file that has its implementation
should be called `some_name_node.py`.
This file should always be executable.
Furthermore, all the logic of the node should be implemented in a Python class
called `SomeNameNode`.

The structure of the `some_name_node.py` should generally look like the following example (without the comments):

```python
#!/usr/bin/env python

# import external libraries
import rospy

# import libraries which are part of the package (i.e. in the include dir)
import library

# import DTROS-related classes
from duckietown.dtros import \
    DTROS, \
    NodeType, \
    TopicType, \
    DTReminder,\
    DTParam, \
    ParamType

# import messages and services
from std_msgs.msg import Float32
from duckietown_msgs.msg import \
    SegmentList, \
    Segment, \
    BoolStamped

class SomeNameNode(DTROS):
    def __init__(self, node_name):
        # class implementation

if __name__ == '__main__':
    some_name_node = SomeNameNode(node_name='same_name_node')
    rospy.spin()
```

Observe that all nodes in Duckietown should inherit from the super class `DTROS`.
This is a hard requirement.
`DTROS` provides a lot of functionalities on top of the standard ROS nodes which
make writing and debugging your node easier, and also sometimes comes with
performance improvements.

In Python code, never ever do universal imports like `from somepackage import *`.
This is a terrible practice.
Instead, specify exactly what you are importing, i.e. `from somepackage import somefunction`.
It is fine if you do it in `__init__.py` files but even there try to avoid it if possible.

When using a package that has a common practice alias, use it, e.g. `import numpy as np`,
`import matplotlib.pyplot as plt`, etc. However, refrain from defining your own aliases.

The code in this node definition should be restricted as much as possible to ROS-related
functionalities.
If your node is performing some complex computation or has any logic that can be separated
from the node itself, implement it as a separate library and put it in the `include`
directory of the package.

## Node initialization

There are a lot of details regarding the initialization of the node so let's
take a look at an example structure of the `__init__` method of our sample node.

```python
class SomeNameNode(DTROS):
    def __init__(self, node_name):
        super(SomeNameNode, self).__init__(
            node_name=node_name,
            node_type=NodeType.PERCEPTION
        )
        
        # Setting up parameters
        self.detection_freq = DTParam(
            '~detection_freq',
            param_type=ParamType.INT,
            min_value=-1,
            max_value=30
        )
        # ...
        
        # Generic attributes
        self.something_happened = None
        self.arbitrary_counter = 0
        # ...

        # Subscribers
        self.sub_img = rospy.Subscriber(
            'image_rect', 
            Image, 
            self.cb_img
        )
        self.sub_cinfo = rospy.Subscriber(
            'camera_info', 
            CameraInfo, 
            self.cb_cinfo
        )
        # ...

        # Publishers
        self.pub_img = rospy.Publisher(
            'tag_detections_image/compressed',
            CompressedImage,
            queue_size=1,
            dt_topic_type=TopicType.VISUALIZATION
        )
        self.pub_tag = rospy.Publisher(
            'tag_detections', 
            AprilTagDetectionArray, 
            queue_size=1,
            dt_topic_type=TopicType.PERCEPTION
        )
        # ...
```

Now, let's take a look at it section by section.

### Node Creation

In classic ROS nodes, you would initialize a ROS node with the function `rospy.init_node(...)`.
DTROS does that for you, you simply need to pass the node name that you want to the super
constructor as shown above.

DTROS supports node categorization, this is useful when you want to visualize the ROS
network as a graph, where graph nodes represent ROS nodes and graph edges represent ROS
topics. In such a graph, you might want to group all the nodes working on the `PERCEPTION`
problem together, say, to clear the clutter and make the graph easier to read.
Use the parameter `node_type` in the super constructor of your node to do so.
Use the values from the `NodeType` enumeration.

Possible node types are the following:

```python
GENERIC
DRIVER
PERCEPTION
CONTROL
PLANNING
LOCALIZATION
MAPPING
SWARM
BEHAVIOR
VISUALIZATION
INFRASTRUCTURE
COMMUNICATION
DIAGNOSTICS
DEBUG
```

### Node Parameters

All parameters should have names relative to the namespace of the node,
i.e. they should start with `~`.
Also, all parameters should be in the scope of the instance, not the method,
so they should always be declared inside the constructor and start with `self.  `.

```{attention}
The parameters should never have default values set in the code.
All default values should be in the configuration file!
```

This makes sure that we don't end up in a situation where there are two different
default values in two different files related to the node.

In classic ROS, you get the value of a parameter with `rospy.get_param(...)`.
One of the issues of the ROS implementation of parameters is that a node cannot request
to be notified when a parameter's value changes at runtime. Common solutions to this
problem employ a polling strategy (which consists of querying the parameter server for
changes in value at regular intervals). This is highly inefficient and does not scale.
The `dtros` library provides a solution to this.
Alternatively to using `rospy.get_param(...)` which simply returns you the current value of
a paramter, you can create a `DTParam` object that automatically updates when a new value is
set.
Use `self.my_param = DTParam("~my_param")` to create a `DTParam` object and
`self.my_param.value` to read its value.

### Generic attributes

Then we initialize all the non-ROS attributes that we will need for this class.
Note that this is done _before_ initializing the Publishers and Subscribers.
The reason is that if a subscriber's callback depends on one of these attributes,
we need to define it before we use it. Here's an example that might fail:

```python
class CoolNode(DTROS):
   def __init__(...):
       self.sub_a = rospy.Subscriber(..., callback=cb_sth, ...)
       self.important_variable = 3.1415

   def cb_sth(self):
       self.important_variable *= 1.0
```

And something that is better:

```python
class CoolNode(DTROS):
   def __init__(...):
       self.important_variable = 3.1415
       sub_a = rospy.Subscriber(..., callback=cb_sth, ...)

   def cb_sth(self):
       self.important_variable *= 1.0
```

### Publishers and Subscribers

Finally, we initialize all the Subscribers and Publishers as shown above.
The `dtros` library automatically decorates the methods `rospy.Publisher` and `rospy.Subscriber`.
By doing so, new parameters are added. All the parameters added by `dtros` have the prefix
`dt_` (e.g., `dt_topic_type`).
Use the values from the `TopicType` enumeration. Possible types list is identical to the
node types list above.

```{note}
Only declare a topic type in a `rospy.Publisher` call.
```

## Naming of variables and functions

All functions, methods, and variables in Duckietown code should be named using `snake_case`. In other words, only lowercase letters with spaces replaced by underscored. Do __not__ use `CamelCase`. This is to be used __only__ for class names.

The names of all subscribers should start with `sub_` as in the example above. Similarly, names of publishers should start with `pub_` and names of callback functions should start with `cb_`.

Initalizing publishers and subscribers should again always be in the scope of the instance, hence starting with `self.`.

## Switching nodes on and off

### Custom behavior on shutdown

If you need to take care of something before when ROS tries to shut down the node, but before it actually shuts it down, you can implement the `on_shutdown` method. This is useful if you are running threads in the background, there are some files that you need to close, resources to release, or to put the robot into a safe state (e.g. to stop the wheels).

## Handling debug topics

Often we want to publish some information which helps us analyze the behavior and performance of the node but which does not contribute to the behavior itself. For example, in order to check how well the lane filter works, you might want to plot all the detected segments on a map of the road. However, this can be quite computationally expensive and is needed only on the rare occasion that someone wants to take a look at it.

A frequent (__but bad design__) way of handling that is to have a topic, to which one can publish a message, which when received will induce the node to start building a publishing the debug message. A much better way, and the one that __should be used__ in Duckietown is to create and publish the debug message _only if_ someone has subscribed to the debug topic.
This is very easy to achieve with the help of `dtros`.
Publishers created within a DTROS node exports the utility function `anybody_listening()`.
Here's an example:

```python
if self.pub_debug_img.anybody_listening():
    debug_img = self.very_expensive_function()
    debug_image_msg = self.bridge.cv2_to_compressed_imgmsg(debug_img)
    self.pub_debug_img.publish(debug_image_msg)
```

Note also that all debug topics should be in the `debug` namespace of the node,
i.e. `~debug/debug_topic_name`.

Similarly, a Subscriber created within a DTROS node exports the utility
function `anybody_publishing()` that checks whether there are nodes that are currently
publishing messages.

## Timed sections

If you have operations that might take non-trivial amount of computational time,
you can profile them in order to be able to analyze the performance of your node.
`DTROS` has a special context for that which uses the same mechanism as the debug topics.
Hence, if you do not subscribe to the topic with the timing information,
there would be no overhead to your performance.
Therefore, be generous with the use of timed sections.

The syntax looks like that:

```python
with self.time_phase("Step 1"):
    run_step_1()

...

with self.time_phase("Step 2"):
    run_step_2()
```

Then, if you subscribe to `~debug/phase_times` you will be able to see for each separate
section detailed information about the frequency of executing it, the average time it takes,
and also the exact lines of code and the file in which this section appears.

## Config files

If your node has at least one parameter, then it should have a configuration file.
If there is a single configuration (as is the case with most nodes) this file should
be called `default.yaml`.
Assuming that our node is called `some_node`, the configuration files for the node
should be in the `config/some_node/` directory.

Every parameter used in the implementation of the node should have a default value
in the configuration file.
Furthermore, there should be no default values in the code.
The only place where they should be defined is the configuration file.

## Launch files

Assuming that our node is called `some_node` then in the `launch` directory of the
package there should be an atomic launch file with the name `some_node.launch` which
launches the node in the correct namespace and loads its configuration parameters.

The launch file content of most node will be identical to the following,
with only the node name and package name being changed.

```xml
<launch>
  <arg name="veh"/>
  <arg name="pkg_name" value="some_package"/>
  <arg name="node_name" default="some_node"/>
  <arg name="param_file_name" default="default" doc="Specify a param file"/>

  <group ns="$(arg veh)">
    <node  name="$(arg node_name)" pkg="$(arg pkg_name)" type="$(arg node_name).py" output="screen">
      <rosparam command="load" file="$(find some_package)/config/$(arg node_name)/$(arg param_file_name).yaml"/>
    </node>
  </group>

</launch>
```
-->