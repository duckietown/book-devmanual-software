```{seo}
:description: Introduction to Docker containerization, images, containers, CPU-architecture tags, and essential commands for efficient Duckietown robotics development.
:keywords: Docker, containerization, containers, images, Duckietown, robotics, Ubuntu, arm32v7, amd64, Portainer, tutorial
```

(sec:developer_basics_docker)=
# Docker

This section introduces Docker and the specific features employed by the Duckietown community. For a broader overview, consult [Docker’s official Get Started guide](https://docs.docker.com/get-started/overview/).

```{tip}
Check out the [Duckietown Introduction to Containerization lecture](https://app-na1.hubspotdocuments.com/documents/8795519/view/717428738?accessId=475a8d) for additional insight. 
```

(basic-docker-what-is)=
## What is Docker?

Docker provides operating-system-level virtualization, commonly called _containerization_. Containers isolate user-space instances so that programs perceive a dedicated environment while sharing the host kernel. This approach yields lightweight, reproducible deployments—ideal for distributed robotics platforms such as the Raspberry Pi used in Duckietown.

In other words, containerization allows the existence of multiple isolated user space instances called containers. These containers may look like real computers from the point of view of programs running in them. 

A computer program running on an ordinary operating system can see all resources available to the system, e.g., network devices, CPU, RAM; However, programs running inside a container can only see the container's resources. Resources assigned to the container become thus available to all processes that live inside that container.

(basic-docker-containers-vs-vms)=
## What is the difference between containers and Virtual Machines?

Containers are often compared to virtual machines (VMs), but there are important differences. Containers differ from virtual machines (VMs) because they _reuse the host kernel_ instead of bundling a full guest operating system. 

|                 | Virtual Machine | Docker Container |
|-----------------|-----------------|------------------|
|Guest OS needed? | Yes             | No               |
|Storage / RAM    | High            | Low              |
|Startup latency  | Seconds-minutes | Milliseconds     |
 
The main difference is that VMs require a host operating system (OS) with a 
hypervisor and a number of guest OSs, each with their own libraries and 
application code. This can result in a significant overhead. 

Imagine running an Ubuntu server inside an Ubuntu VM. Most 
of the kernel libraries, binaries, and processes will be duplicated on the host and on the guest OS. 
Containerization, on the other hand, leverages the existing kernel and OS 
and adds only the additional binaries, libraries, and code necessary to run 
a given application.

```{figure} ../../_images/basics/development/containers_vs_vms.jpg
:name: fig-containers-vs-vms
:alt: Diagram comparing containers and virtual machines
:align: center
:width: 90%

Differences between virtual machines and containers (source: Weaveworks).
```

Containers do not need a separate OS to run, making them more lightweight than VMs.

This makes them perfect to use in cases where one needs to deploy a lot of independent services on the same hardware, or to deploy on devices with limited computational resources (compared to most desktop or laptop computers), such as a Raspberry Pi or Jetson Nano used in Duckietown.

Containers allow for reuse of resources and code, but also simplify the workflow in the context of version control. When using a VM, changes need to be updated from within the VM. With a Docker container, the same process just requires pulling the container _image_ again.

(basic-docker-how-works)=
## How does Docker work?

Docker uses three main entities: images → layers → containers. Containers are built from images, which are made of layers.

* Image: build-time artifact
* Layer: incremental filesystem change inside an image
* Container: run-time instance of an image

### Docker images, layers, and containers

Docker images are _build-time_ artifacts while Docker containers are _run-time_ constructs. 
In other words, a Docker image is static, like a `.zip` or `.iso` file. A container, on the other hand, starts from a static image but as it is used, files and configurations might change.

Docker images are built from layers. The initial, or lowest-level one being the *base layer*. Typically, this is a stripped-down version of an OS. For example, many Docker images in Duckietown run `ubuntu:18.04` as a base.

Each layer on top of the base layer represents a change to the layers below. Docker translates this sequence of changes to a file system that the container can then use. 

Small changes to a single file typically only modify one layer. Therefore, when Docker attempts to pull the new version, it will need to download and store only the modified layer, saving space, time, and bandwidth.

### Docker Hub and image tags

In the Docker world, images are organized by their repository name, image 
name, and _tags_. As with Git and GitHub, Docker images are stored in image 
_registers_. The most popular Docker register, and the one used in Duckietown, [Docker Hub](https://hub.docker.com/). Docker Hub organizes images as:

    [repository/]image[:tag]

The parts `repository` and `tag` are optional, and default to
`library` (indicating Docker official images) and `latest` (special
tag always pointing to the _latest_ version of an image).

For example, in the Duckietown Docker image:

    duckietown/dt-core:ente-arm64v8

* repository name `duckietown`, 
* image name `dt-core`, and
* tag `ente-arm64v8`, 

The tag specifies the Duckietown software distribution that the image contains, i.e., `ente`, and the CPU architecture that this image is targeting, i.e., `arm64v8`. We will talk about different CPU architectures and why they need to be part of the Docker image tag in [](basic-docker-arch).

All Duckietown-related images are in the `duckietown` Docker Hub repository, although images can be very different from each other and for used for various 
applications.

(basic-docker-arch)=
## CPU-Architecture Tags

Docker images contain binaries, executable files that are compiled to
the level of CPU instructions, which are not portable across
different CPU architectures, due to different instruction sets.

Most modern computers with Intel or AMD CPUs use the `amd64` architecture, so it is possible to never have to worry about CPU architecture distinctions. 

But in Duckietown we use "edge" devices like the Raspberry Pi and NVIDIA Jetson Nanos. These small computers employ ARM processors, based on the `arm32v7` architecture.

<!--
Note: Full disclosure, while all devices officially supported in Duckietown
are based on 64-bit capable Arm processors, thus using the `arm64v8` 
instructions set, the Raspbian OS only supports 32-bit, which is the reason 
why we use `arm32v7` images.
-->

(basic-docker-commands-images)=
## Working with images

```
docker pull ubuntu            # download, e.g., the ubuntu image
docker image list             # list local images
docker image rm ubuntu        # remove the ubuntu image
docker image prune            # remove all unused images
```

Images are obtained from registries like Docker Hub through `pull` commands.
E.g., to get an Ubuntu image:

    docker pull ubuntu

As mentioned in [](basic-docker-how-works), this will pull the image with full name `library/ubuntu:latest` which, as of April 2025, corresponds
to Ubuntu 24.04.

To view all images in the system, including the ubuntu one just pulled, use:

    docker image list

To remove an image instead:

    docker image rm ubuntu

| REPOSITORY       | TAG    | IMAGE ID | CREATED        | SIZE   |
|------------------|--------|-------|----------------|--------|
| ubuntu | latest | 602eb6fb314b | 30 minutes ago | 78.1MB |

You can also remove images by their `IMAGE ID` as printed by the 
`list` command above. A shortcut for `docker image rm` is `docker rmi`.

To remove all "dangling images", i.e., those not supporting any container:

    docker image prune

Images that are being used by one or more containers cannot be removed without removing those containers first.

To look into the heart and soul of images, use the commands:

    docker image history
    docker image inspect

(basic-docker-commands-containers)=
## Working with containers

### Essential container commands

```
docker run -it ubuntu         # interactive shell
docker ps                     # running containers
docker container list -a      # all containers
docker container rm <id>      # remove container
docker container start <id>   # start
docker container stop  <id>   # stop
docker attach <id>            # attach shell
```

Containers are the run-time equivalent of images. To start a container, Docker:
* picks up the specified image,
* creates a file system from its layers,
* attaches all the specified devices and directories,
* "boots" it up,
* sets up the environment, and
* starts a pre-determined process inside this container. 

All this magic happens with the simple command: `docker run`. 

```{note}
It is not necessary to pull the image layers beforehand. If Docker does not find them locally, it will look for them on Docker Hub.
```

### Working with containers - an example

Here is an example:

    docker run ubuntu  

This will take the `ubuntu` image with `latest` tag (previously pulled - `docker pull ubuntu` again if it has been removed) and will start a container from it.

This command will not do much, as the container will immediately exit as it has nothing to execute. When the main process of a container exits, the container exits as well. 

By default, the `ubuntu` image runs `bash`. As we have not specified any commands, it exits immediately.

The container can be kept active by using the `-it` flags, telling Docker to create an interactive session.

    docker run -it ubuntu

The terminal will now show something similar to:

    root@73335ebd3355:/#

The part after `@` will be different --- that is your container ID.

We are now inside the `ubuntu` container and can use basic `bash` commands like `ls`, `cd`, and `cat`, and verify we no longer have access to the host machine through this terminal.  

Running containers can be listed with the `docker ps` command --- analogous to the `docker image list` command for images. 

Open a new terminal window (do not close the other one yet) and type:

    docker ps

An alternative (more explicit) syntax is

    docker container list

Go back to the terminal running the `ubuntu` container and type `exit`. This will stop the container and revert control over to the host machine. 
Running the `docker ps` command again will show that the ubuntu container disappeared. 

Does this mean that this container and all changes you might have made in it are gone? 

Not at all, `docker ps` and `docker container list` only list the *currently running* containers.

You can see all containers, including the stopped ones with:

    docker container list -a

(or, `docker ps -a`). Here `-a` stands for *all*. 

At least two `ubuntu` containers should show here, because every time `docker run` is used a new container is created. 

```{note}
The container `NAMES` seem strangely random. We could have added custom, more descriptive names --- more on this later.
```

We do not really need these containers, so let us remove them:

    docker container rm [container name 1] [container name 2]

You need to specify the container names after `rm`, or the container IDs. Note that if the container is still running it will be necessary to stop it before removing it.

Other common container operations are intuitive:

    docker container start [container name]
    docker container stop [container name]
    docker container restart [container name]

```{attention}
Finally, a powerful command. Imagine running a container in the background, with the main process running but no open shell. How to interact with what is going on inside the container? 
```

Open a terminal in the container with:

    docker attach [container name]

(basic-docker-running-options)=
## Running images

Often it will be necessary to run containers with more sophisticated 
options. Look at the following example: (do not try to run it, it will not do much).

    docker -H hostname.local run -dit --privileged --name joystick --network=host -v /data:/data duckietown/rpi-duckiebot-joystick-demo:master18

[](tab:docker-run-tab) summarizes the options commonly used in Duckietown.

:::{list-table} `docker run` Options
:header-rows: 1
:name: tab:docker-run-tab
:widths: 10, 20, 70

* - Short command
  - Full command
  - Explanation
* - `-i`
  - `--interactive`
  - Keep STDIN open even if not attached, typically used together with `-t`.
* - `-t`
  - `--tty`
  - Allocate a pseudo-TTY, providing terminal access to the container, typically used together with `-i`.
* - `-d`
  - `--detach`
  - Run container in the background and print container ID.
* -
  - `--name`
  - Sets a name for the container. If you don't specify one, a random name will be generated.
* - `-v`
  - `--volume`
  - Bind mount a volume, exposes a folder on your host as a folder in your container. Be vigilant when using this.
* - `-p`
  - `--publish`
  - Publish a container's port(s) to the host, necessary when communication is needed with a program in the container.
* - `-d`
  - `--device`
  - Similar to `-v` but for devices. This grants the container access to a specified device. Be cautious when using this.
* -
  - `--privileged`
  - Give extended privileges to this container. That includes access to **all** devices. Be **extremely** careful when using this.
* -
  - `--rm`
  - Automatically remove the container when it exits.
* - `-H`
  - `--hostname`
  - Specifies a remote host name, e.g., when executing the command on a Duckiebot and not on your base station.
* -
  - `--help`
  - Prints information about these and other options.

:::


### Running images - Examples

Set the container name to `joystick`:

    --name joystick

Mount the host's path `/home/myuser/data` to `/data` inside the container:

    -v /home/myuser/data:/data

Publish port 8080 in the container as 8082 on the host:

    -p 8082:8080

Allow the container to use the device `/dev/mmcblk0`:

    -d /dev/mmcblk0

Run a container on the Duckiebot:

    -H duckiebot.local

What is the outcome expectation?

## Other useful commands

### Pruning images

Sometimes your Docker system will be clogged with images, containers, networks, etc. It can be cleaned up with:

    docker system prune


```{warning}
`docker system prune` will delete **all** containers that are not currently running and **all** images not used by running containers. Use with caution.
```

### Portainer

For simple operations and basic commands, one can use _Portainer_.

Portainer is a Docker container that allows control of the Docker 
daemon through a browser. Portainer can be installed by running:

    docker volume create portainer_data
    docker run -d -p 9000:9000 --name portainer --restart always -v /var/run/docker.sock:/var/run/docker.sock -v portainer_data:/data portainer/portainer

Note that Portainer comes pre-installed on your Duckiebot, and can be accessed through the [Duckietown Dashboard](book-opmanual-duckiebot:duckiebot-dashboard-setup).


## Hands-On activities

Before doing any software development in Duckietown, get
comfortable with Docker and its tools. Complete these Docker tutorials before proceeding to the next section:

1. [Install Docker](https://docs.docker.com/get-docker/)
2. [Orientation and Setup](https://docs.docker.com/get-started/)
3. [Build and run your image](https://docs.docker.com/get-started/part2/)
4. [Share images on Docker Hub](https://docs.docker.com/get-started/part3/)

Additional insights fore the curious learner can be found in this 
["Docker Curriculum" guide](https://docker-curriculum.com/).


## Ask the community

If you have any questions about how to use of Docker in Duckietown,
[join the Duckietown Slack community](https://duckietown.com/join-slack) and ask away.
