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

