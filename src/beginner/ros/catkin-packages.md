(ros-catkin-packages)=
# Catkin Packages

```{needget}
* New ROS DTProject as described [here](ros-project-create-new)
---
* Learn how to create a new [catkin](http://wiki.ros.org/catkin) package inside a DTProject
```

ROS uses the [catkin](http://wiki.ros.org/catkin) build system to organize and build its software.
If you are not familiar with the catkin build system, you can learn about it by following the
[official tutorials](http://wiki.ros.org/catkin/Tutorials).

In a nutshell, catkin organizes entire projects in the so-called _catkin workspaces_.
A catkin workspace is nothing more than a directory containing a bunch of software modules called
_catkin packages_. Each software module can contain a set of executables (e.g., binaries, script files)
called _ROS nodes_. ROS nodes interact with one another using two of the most common communication patterns,
called 
[`publish-subscribe`](https://en.wikipedia.org/wiki/Publish%E2%80%93subscribe_pattern) and 
[`request-reply`](https://en.wikipedia.org/wiki/Request%E2%80%93response).

ROS implements the `publish-subscribe` pattern using **ROS Publishers** and **ROS Subscribers**, and the
`request-reply` pattern using **ROS Services**. More on these later.

Let us now take a step back and review catkin workspaces, packages, and nodes more in details.


(ros-catkin-workspace)=
## Catkin workspace

The directory `packages/` you find at the root of a DTProject is a catkin workspace.

````{admonition} Advanced: the real story behind the _packages_ directory
:class: dropdown

In reality, `packages/` is more of a sub-workspace, as it is internally joined with other `packages/`
directories from other projects (the ancestor projects) to form a full catkin workspace.
````


(ros-catkin-package-create)=
## Create a new Catkin package

Open a terminal at the root of the DTProject `my-ros-project` created earlier. 
Again, Catkin packages are directories inside the directory `packages/` of `my-ros-project`. 
Let us go ahead and create a new directory called `my_package` inside `packages/`.

    mkdir -p ./packages/my_package

A Catkin package (also known as a _ROS package_) is simply a directory containing two special files, 
`package.xml` and `CMakeLists.txt`. 
So, let us turn the `my_package` folder into a ROS package by creating these two files.

Create the file `package.xml` inside `my_package` using your favorite text editor and 
place/adjust the following content inside it:

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

Replace `YOUR_FULL_NAME` with your first and last name and `YOUR_EMAIL@EXAMPLE.COM` with your email address.

---

Now, create the file `CMakeLists.txt` inside `my_package` using your favorite text editor and 
place/adjust the following content inside it:

```cmake
cmake_minimum_required(VERSION 2.8.3)
project(my_package)

find_package(catkin REQUIRED COMPONENTS
  rospy
)

catkin_package()
```

We now have a catkin package inside a catkin workspace in a ROS-capable DTProject.
We will now add our code by adding ROS nodes to our catkin package.
