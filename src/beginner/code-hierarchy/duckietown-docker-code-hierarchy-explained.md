```{seo}
:description: Understand the Docker image hierarchy and code repositories used in the Duckietown ecosystem, from base environments to high‑level demos.
:keywords: Duckietown, Docker images, dt-core, dt-commons, dt-ros-commons, dt-duckiebot-interface, dt-car-interface, container hierarchy
```
(code-hierarchy)=
# Beginner - Code Hierarchy
```{needget}
* An understanding of the basics of [Docker](book-opmanual-duckiebot:preliminaries-docker-basics)
* An initialized [Duckiebot](book-opmanual-duckiebot:get-db-hw)
---
* Knowledge of the software architecture on a Duckiebot
```
To develop new functionality within Duckietown, it is essential to understand the existing code structure. This module introduces the top‑level layout and points to resources for deeper exploration.

While Duckietown may appear as a simple toy car platform, it supports cars, boats, drones, and runs on real robots, in simulation, or in an evaluator / competition setup, as previously done with the AI Driving Olympics settings. This module focuses on the code that powers standard demos (e.g., Lane Following, Indefinite Navigation).

(docker-images)=
## Main images and repositories

Three images frequently appear when running demos or tools: `dt-duckiebot-interface`, `dt-car-interface`, and `dt-core`. Each inherits from a common base. The diagram below illustrates this hierarchy.

```{figure} ../../_images/beginner/container_stack.png
:name: container-stack
:alt: Docker image hierarchy in Duckietown

The Docker image hierarchy.
```

All three of the containers actually inherit the same container. Recall that 'inheritance' in a Docker images means that the 'child' image has a `FROM` statement with the 'parent' image in its Dockerfile. Hence, 'child' _is based on 'parent'_.

The image on which everything is based is `ubuntu`. This is the official Ubuntu image, with no additions. While the `daffy` distribution of Duckietown uses Ubuntu 18.04, `ente` uses Ubuntu 22.04 (Jammy Jellyfish). This parent image needs to be properly configured, and features added, to use Duckietown software.

The `duckietown/dt-base-environment` adds many useful core libraries and configurations. E.g., development tools such as `vim`, `git`, `nano` and libraries for handling `i2c` devices, processing images, and efficiently doing linear algebra. It moreover adds compilers, linkers, and libraries necessary for the compiling/building of software from source. Furthermore, we add `pip` and a bunch of handy `python3` libraries, such as `numpy`, `scipy`, `matplotlib`, and `smbus` (used to communicate with motors, LEDs, etc.). Finally, `duckietown/dt-base-environment` also provides the core ROS libraries, including `rospy`: ROS's Python bindings. The version of `ROS` used is [ROS Noetic Ninjemys](http://wiki.ros.org/noetic).

Then, `duckietown/dt-commons` builds on top of `duckietown/dt-base-environment`. We provide a number of Duckietown libraries here that deal with the handling of files, infrastructure communication, and other development tools. This image also configures the environment so that the hostname resolution is correctly performed and ensures that the environment variables pertaining to the type of the robot, its hardware, and configuration are all properly set. It also makes sure that all Python libraries are discoverable and that ROS is set up correctly.

Building on is `duckietown/dt-ros-commons`, which has everything needed to 
develop code that directly works on Duckiebots. However, as there are a few components that all Duckietown ROS nodes share, it is convenient to package them in an image. These are `duckietown-utils` (a library with a number 
of useful functions), `duckietown_msgs` (a ROS package that contains all the ROS message types used in Duckietown), and `DTROS`. `DTROS` is a 'mother' node for all other nodes in Duckietown. You have already seen it while working with ROS publishers and subscribers, but we will look at it in more detail soon.

The `duckietown/dt-ros-commons` is also the place where we keep protocols that are key for the communication between nodes found in different repositories. By placing them here, we ensure that all repositories work with the exact same protocol, and hence we prevent communication issues. Currently, the only protocol there is `LED_protocol`, which is used by the `led_emitter_node` in `dt-duckiebot-interface`, which emits LED-encoded messages, and by the `led_detector_node` in `dt-core`, which interprets the messages encoded in the LED flashing of other robots.

Finally, `duckietown/dt-ros-commons` packs another handy node: the `ros_http_api_node`. It exposes the ROS environment as an HTTP API. The ROS HTTP API runs by default on any Duckietown device and allows access to ROS topics, parameters, services, nodes, etc., over HTTP, which is an extremely portable interface. We use this, e.g., in the Duckietown Dashboard.

We finally can focus on `dt-duckiebot-interface`, `dt-car-interface`, and `dt-core`. The first, `dt-duckiebot-interface`, contains all the hardware drivers for the Duckiebot. In particular these are the drivers for the camera (in the `camera_driver` package), motors (`wheels_driver`), and LED drivers (`led_emitter`). Thanks to these nodes, it is not necessary to interact with low-level code to control a Duckiebot. Instead, once can use the convenient ROS topics and services provided by these nodes.

The `dt-car-interface` image provides additional basic functionality not on hardware level. It includes what is needed to drive a Duckiebot, in particular the parts that handle the commands sent by a (virtual) joystick (the `joy_mapper` package) and the forward and inverse kinematics that convert the desired robot movement to wheel commands (`dagu_car` package). It might not be immediately clear at first why these are not part of `dt-duckiebot-interface` or `dt-core`. In some use cases, e.g., for the demos or controlling a when robot via keyboard or joystick, it is beneficial to have these two packages. For others cases, e.g., when deploying a completely different pipeline (think end-to-end reinforcement learning), one would prefer to interact directly with the drivers. 

The `dt-core` image provides all the high-level robot behavior observed when running a demo. The image processing pipeline, decision-making modules, lane and intersection contollers, and many others reside there.

If you are curious to see all the ROS packages available in each of these images, you can check out the corresponding GitHub repositories:

- [`dt-base-environment`](https://github.com/duckietown/dt-base-environment/tree/ente)

- [`dt-commons`](https://github.com/duckietown/dt-commons/tree/ente)

- [`dt-ros-commons`](https://github.com/duckietown/dt-ros-commons/tree/ente)

- [`dt-duckiebot-interface`](https://github.com/duckietown/dt-duckiebot-interface/tree/ente)

- [`dt-car-interface`](https://github.com/duckietown/dt-car-interface/tree/ente)

- [`dt-core`](https://github.com/duckietown/dt-core/tree/ente)

```{note}
Make sure to look at the `ente` branches of these repositories! This is the most current release of the Duckietown software. You can switch to the `daffy` version from the links above if you are using teh `daffy` Duckietown distribution.
```

Inline documentation is provided within the code.

```{warning}
At the moment only `dt-ros-commons`, `dt-duckiebot-interface`, and `dt-car-interface` are documented. We are working on providing similar level of documentation for `dt-core` as well.
```

## Various configurations of the Duckietown codebase

The Duckietown codebase can be used in various configurations: e.g., on a physical robot, in simulation, or as a submission to a challenge. Different use cases are best served by using different images. Here are some pointers:

### Driving with a (virtual) joystick

If you only want to drive your Duckiebot around, you need the `joy_mapper` node that translates the joystick `Joy` messages to car command messages, the `kinematics` node that in turn converts these to wheel command messages, and the `wheels_driver` node that controls the motors. So the `dt-duckiebot-interface` and `dt-car-interface` images are enough.

```{figure} ../../_images/beginner/config_joystick.png
:name: joystick
:alt: Duckietown virtual joystick

Driving a Duckiebot with a virtual joystick.
```
<!--
### Driving through the Dashboard

```{warning}
This functionality has been disabled.
```

As you saw when setting up your Duckiebot, the Dashboard and the Compose interface also provide manual driving 
functionality. For this, one needs the same images as before, of course together with the Dashboard image itself:

```{figure} ../../_images/beginner/config_dashboard.png
:name: dashboard

Driving through the Dashboard.
```
-->

### Running a demo on a Duckiebot

Running a demo requires driving around together with the high-level processing and logic that reside in the `dt-core` image:

```{figure} ../../_images/beginner/config_demo_bot.png
:name: dt-demo

Running a demo on a Duckiebot.
```

### Running a demo in simulation

A demo can also be executed in simulation. In this case, instead of using the hardware drivers `dt-duckiebot-interface` provides, we substitute them with the simulator interface:

```{figure} ../../_images/beginner/config_demo_sim.png
:name: demo-sim

Running a demo in simulation.
```


### Evaluating submissions to challenges in simulation

A submission, e.g., for a MOOC exercises or the former AI Driving Olympics, is a container that receives image data and outputs wheel commands. Therefore, it can replace the `dt-car-interface` and `dt-core` images and still use the same simulator framework. This can also be done in the cloud, and that is exactly one of the roles of the [Duckietown Challenges server](https://challenges.duckietown.org/v4).

```{figure} ../../_images/beginner/config_aido_sim.png
:name: aido-sim

Evaluating a submission in simulation.
```


### Evaluating submissions on a Duckiebot

The same submission image, with not a single change, can be also tested on a physical Duckiebot by substitute the simulator with `dt-duckiebot-interface`. As the containers do not need to necessarily run on the same device, we can also use much powerful computers (also state-of-the-art GPUs) when testing submissions. 

<!--
This is the way that AI-DO submissions are evaluated in Autolabs. 

In this way, even if you do not have a Duckiebot, you can develop your submission in simulation, then submit it to be evaluated in simulations on the challenges server, and if it performs well, you can request remote evaluation on a real Duckiebot in a Duckietown Autolab!
-->

```{figure} ../../_images/beginner/config_aido_bot_separated.png
:name: dt-aido

Evaluating a submission on a Duckiebot.
```