```{seo}
:description: Learn tricks for a faster development workflow with DTProjects and `dts`, including running ROS nodes locally or remotely.
:keywords: Duckietown, DTProject, ROS, development workflow, dts, remote run, local run, optimization
```
(dtproject-ros-faster-development)=
# Faster Development Workflow
In this section, some tricks for a much faster development workflow using `dts` are presented.

(dtproject-ros-faster-development-trick-run-locally)=
## Run locally

As demonstrated previously, it is possible to add new code to a DTProject or modify existing code and then build and run either locally or directly on a Duckiebot. While working with ROS, examples so far have executed all nodes on the Duckiebot. 

This is because ROS configures nodes to default to the ROS network on the _local_ machine, and given that the ROS network originates on the Duckiebot, this default configuration can be leveraged by running the nodes directly on the Duckiebot.

However, building and running on the Duckiebot is not optimal for speed, and a responsive development workflow is crucial.

There are two major issues with the current workflow:

1. The source code resides on the local computer, so Docker must transfer it to the Duckiebot to build the image.
2. The Duckiebot’s on-board computer is too slow to serve as a rapid development testbed.

3. Ideally, it should be possible to build and run ROS nodes on the local computer in a manner transparent to all other ROS nodes. This can be achieved easily with `dts`.

Consider the example in [](ros-sub-node). A block diagram showing the ROS nodes and their location in the network would be as follows:

```{figure} ../../_images/beginner/ros/dts_devel_ros_remote.png
:name: fig:dts-devel-ros-pub-sub-remote
:alt: ROS Pub-Sub on Duckiebot remote
:align: center
:width: 90%
Block diagram for a Pub-Sub setup with both nodes running on the Duckiebot.
```

Now, maintain everything as is for the Publisher and adjust the build and run commands for the Subscriber as follows:

```bash
dts devel build -f
dts devel run -R ROBOT_NAME -L my-subscriber
```

Here, `dts` is instructed to build the project locally (omitting `-H ROBOT_NAME`) and to run the subscriber node locally (omitting `-H ROBOT_NAME`) while connecting it to the Duckiebot’s ROS network using the `--ros` (`-R`) option.

A block diagram showing the new configuration of ROS nodes and their location in the network would be:

```{figure} ../../_images/beginner/ros/dts_devel_ros_local.png
:name: fig:dts-devel-ros-pub-sub-local
:alt: ROS Pub-Sub mixed local and remote
:align: center
:width: 90%
Block diagram for a Pub-Sub setup with the Subscriber node running on the local computer.
```


<!--
(dtproject-ros-faster-development)=
# Faster Development Workflow

In this section we will learn some tricks that enable a much faster development workflow in `dts`.


(dtproject-ros-faster-development-trick-run-locally)=
## Run locally

As we have seen so far, we can add new code to a DTProject or make a change to existing code and then build 
and run either locally or directly on a Duckiebot. While working with ROS, we only have seen examples in which 
all the nodes were run on the Duckiebot. This is because ROS sets up all the nodes to defaultly look for a 
ROS network on the _local_ machine, and given that our ROS network originates on the Duckiebot, we can leverage 
the default configuration of ROS by running the nodes directly on the Duckiebot. 
Unfortunately, building and running on the Duckiebot is not the best option
when it comes to speed, though having a responsive development workflow is crucial in software development.

There are two major issues with the current workflow,
1. our source code always resides on our local computer, so Docker needs to transfer it over to the Duckiebot for the image to be built;
2. the Duckiebot's on-board computer is too slow to be used as a development testbed (while it is fine for final deployments);

Ideally, we would like to be able to build and run ROS nodes on our local computer in a way that is transparent
to all other ROS nodes. This can be done very easily with `dts`, and we will now see how.

Let us go back to the example in [](ros-sub-node). A block diagram showing the ROS nodes and their location in 
the network would be the following,

```{figure} ../../_images/beginner/ros/dts_devel_ros_remote.png
:width: 100%
:name: fig:dts-devel-ros-pub-sub-remote

Block diagram for a Pub-Sub setup with both nodes running on the Duckiebot.
```

Let us now keep everything as is for the Publisher and slightly change the commands we use to build and 
run the Subscriber.
In particular, we use the following commands instead,

    dts devel build -f
    dts devel run -R ROBOT_NAME -L my-subscriber

We are now telling `dts` to build the project locally (we removed `-H ROBOT_NAME` from the `build` command).
We are also telling `dts` to run the subscriber node locally (we removed `-H ROBOT_NAME` from the 
`run` command) but to connect it to the ROS network of the Duckiebot (using the `--ros/-R ROBOT_NAME` 
option on the `run` command).
A block diagram showing the new configuration of ROS nodes and their location in
the network would be the following,


```{figure} ../../_images/beginner/ros/dts_devel_ros_local.png
:width: 100%
:name: fig:dts-devel-ros-pub-sub-remote

Block diagram for a Pub-Sub setup with the Subscriber node running on the local computer.
```
-->
