```{seo}
:description: Learn how to create and configure Catkin packages within a ROS-compatible Duckietown DTProject using the Catkin build system.
:keywords: Duckietown, ROS, Catkin, DTProject, catkin workspace, ROS packages, robotics development
```

(ros-catkin-packages)=
# Catkin Packages

```{needget}
* New ROS DTProject as described [here](ros-project-create-new)
---
* Learn how to create a new [catkin](http://wiki.ros.org/catkin) package inside a DTProject
```

ROS uses the [catkin](http://wiki.ros.org/catkin) build system to organize and build its software.
IIf unfamiliar with catkin, follow the [official tutorials](http://wiki.ros.org/catkin/Tutorials).

In a nutshell, catkin organizes entire projects in the so-called _catkin workspaces_, which are directories containing software modules called
_catkin packages_. Each module can include executables (e.g., binaries, script files) known as _ROS nodes_. Nodes interact with one another using two of the most common communication patterns, called [`publish-subscribe`](https://en.wikipedia.org/wiki/Publish%E2%80%93subscribe_pattern) and [`request-reply`](https://en.wikipedia.org/wiki/Request%E2%80%93response).

ROS implements the `publish-subscribe` pattern using **ROS Publishers** and **ROS Subscribers**, and the `request-reply` pattern using **ROS Services**.

(ros-catkin-workspace)=
## Catkin workspace

The `packages/` directory at the root of a DTProject serves as the catkin workspace.

````{admonition} Advanced: the real story behind the _packages_ directory
:class: dropdown

The `packages/` directory is actually a sub-workspace. It merges with ancestor projects’ `packages/` directories to form a complete catkin workspace.
````


(ros-catkin-package-create)=
## Create a new Catkin package

Open a terminal at the root of the DTProject `my-ros-project`. Catkin packages are directories inside the directory `packages/` of `my-ros-project`. Create a new package directory:

    mkdir -p ./packages/my_package

A Catkin package (also known as a _ROS package_) requires two files: 
`package.xml` and `CMakeLists.txt`.

1. Create `packages/my_package/package.xml` with the following content:

```xml
<package>
  <name>my_package</name>
  <version>0.0.1</version>
  <description>
  My first Catkin package in Duckietown.
  </description>
  <maintainer email="YOUR_EMAIL@EXAMPLE.COM">YOUR_FULL_NAME</maintainer>
  <license>None</license>

  <buildtool_depend>catkin</buildtool_depend>
</package>
```

Replace `YOUR_FULL_NAME` and `YOUR_EMAIL@EXAMPLE.COM` accordingly.

2. Create `packages/my_package/CMakeLists.txt` with the content:

```cmake
cmake_minimum_required(VERSION 2.8.3)
project(my_package)

find_package(catkin REQUIRED COMPONENTS
  rospy
)

catkin_package()
```

We now have a Catkin package inside a Catkin workspace in a ROS-capable DTProject. We can now proceed to add ROS nodes. 
