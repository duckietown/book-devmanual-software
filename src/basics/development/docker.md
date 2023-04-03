(sec:developer_basics_docker)=
# Docker

This section will introduce Docker and the features of Docker that the
Duckietown community employs. For a more general introduction to Docker, 
we suggest reading the official 
[Docker overview](https://docs.docker.com/get-started/overview/)
page.


(basic-docker-what-is)=
## What is Docker?

Docker is used to perform operating-system-level virtualization, something 
often referred to as "containerization". While Docker is not the only 
software that does this, it is by far the most popular one.

Containerization refers to an operating system paradigm in which the kernel 
allows the existence of multiple isolated user space instances called containers.
These containers may look like real computers from the point of view of programs
running in them. 

A computer program running on an ordinary operating system can see all resources
available to the system, e.g. network devices, CPU, RAM; However, programs 
running inside of a container can only see the container's resources. Resources
assigned to the container become thus available to all processes that live inside
that container.


(basic-docker-containers-vs-vms)=
## Containers VS. Virtual Machine

Containers are often compared to virtual machines (VMs). 
The main difference is that VMs require a host operating system (OS) with a 
hypervisor and a number of guest OSs, each with their own libraries and 
application code. This can result in a significant overhead. 

Imagine running a simple Ubuntu server in a VM on Ubuntu: you will have most 
of the kernel libraries and binaries twice and a lot of the processes will 
be duplicated on the host and on the guest OS. 
Containerization, on the other hand, leverages the existing kernel and OS 
and adds only the additional binaries, libraries and code necessary to run 
a given application. See the illustration bellow.


```{figure} ../../_images/basics/development/containers_vs_vms.jpg
:width: 32em
:name: figure-example-2
:alt: Containers VS Virtual Machines

Differences between Virtual Machines and Containers (source: Weaveworks)
```

Because containers don't need a separate OS to run they are much more 
lightweight than VMs. This makes them perfect to use in cases where one needs 
to deploy a lot of independent services on the same hardware or to deploy on 
not-especially powerful platforms, such as Raspberry Pi - the platform the
Duckietown community uses.

Containers allow for reuse of resources and code, but are also very easy to 
work with in the context of version control. If one uses a VM, they would need 
to get into the VM and update all the code they are using there. With a 
Docker container, the same process is as easy as pulling the container 
image again.


(basic-docker-how-works)=
## How does Docker work?

You can think that Docker containers are build from Docker images which in 
turn are build up of Docker layers. So what are these?

Docker images are build-time artifacts while Docker containers are run-time 
constructs. That means that a Docker image is static, like a `.zip` or `.iso` 
file. A container is like a running VM instance: it starts from a static 
image but as you use it, files and configurations might change.

Docker images are build up from layers. The initial layer is the *base layer*, 
typically an official stripped-down version of an OS. For example, a lot of 
the Docker images we run in Duckietown have `ubuntu:18.04` as a base.

Each layer on top of the base layer constitutes a change to the layers below. 
The Docker internal mechanisms translate this sequence of changes to a file 
system that the container can then use. If one makes a small change to a file, 
then typically only a single layer will be changed and when Docker attempts to 
pull the new version, it will need to download and store only the changed layer, 
saving space, time and bandwidth.

In the Docker world, images get organized by their repository name, image 
name and tags. As with Git and GitHub, Docker images are stored in image 
registers. The most popular Docker register is called DockerHub and it is 
what we use in Duckietown.

An image stored on DockerHub has a name of the form:

    [repository/]image[:tag]

The parts `repository` and `tag` are optional and they default to
`library` (indicating Docker official images) and `latest` (special
tag always pointing to the _latest_ version of an image).
For example, the Duckietown Docker image

    duckietown/dt-core:daffy-arm32v7

has the repository name `duckietown`, 
the image name `dt-core`, 
and the tag `daffy-arm32v7`, which carries both the name of the 
Duckietown software distribution that the image contains, i.e., `daffy`,
and the CPU architecture that this image is targeting, i.e., `arm32v7`.
We will talk about different CPU architectures and why they need to be
part of the Docker image tag in the section [](#basic-docker-arch).

All Duckietown-related images are in the `duckietown` repository. 
Though images can be very different from each other and for various 
applications.


(basic-docker-arch)=
## Different CPU architectures

Since Docker images contain binaries, they are not portable across
different CPU architectures.
In particular, binaries are executable files that are compiled to
the level of CPU instructions.
Different CPU architectures present different instructions sets.

Many modern computers use the `amd64` architecture, used by almost
all modern Intel and AMD processors. This means that it is very likely
that you can find a Docker image online and run it on your computer
without having to worry about CPU architectures.

In Duckietown, we use low-end computers like the Raspberry Pi (officially
used on any Duckietown device) and Nvidia Jetson. These low-cost computers
employ Arm processors that are based on the `arm32v7` instructions set.

Note: Full disclosure, while all devices officially supported in Duckietown
are based on 64-bit capable Arm processors, thus using the `arm64v8` 
instructions set, the Raspbian OS only supports 32-bit, which is the reason 
why we use `arm32v7` images.


(basic-docker-commands-images)=
## Working with images

If you want to get a new image from a Docker registry (e.g. DockerHub), 
you have to *pull* it. For example, you can get an Ubuntu image by running 
the command:

    docker pull ubuntu

According to [](#basic-docker-how-works), this will pull the image
with full name `library/ubuntu:latest` which, as of May 2020, corresponds
to Ubuntu 20.04.

You will now be able to see the new image pulled by running:

    docker image list

If you don't need it anymore or you are running out of storage space, 
you can remove an image with,

    docker image rm ubuntu

You can also remove images by their `IMAGE ID` as printed by the 
`list` command above. A shortcut for `docker image rm` is `docker rmi`.

Sometimes you might have a lot of images you are not using anymore. 
You can easily remove them all with:

    docker image prune

This will remove all images that are not supporting any container. In fact, 
you cannot remove images that are being used by one or more containers.
To do so, you will have to remove those containers first.

If you want to look into the heart and soul of your images, you can use the commands 
`docker image history` and `docker image inspect` to get a detailed view.


(basic-docker-commands-containers)=
## Working with containers

Containers are the run-time equivalent of images. When you want to start a container, 
Docker picks up the image you specify, creates a file system from its layers, attaches 
all devices and directories you want, "boots" it up, sets up the environment up and 
starts a pre-determined process in this container. 
All that magic happens with you running a single command: `docker run`. 
You don't even need to have pulled the image beforehand, if Docker can't find it locally, 
it will look for it on DockerHub.

Here's a simple example:

    docker run ubuntu

This will take the `ubuntu` image with `latest` tag and will start a container from it.

The above won't do much. In fact, the container will immediately exit as it has nothing 
to execute. When the main process of a container exits, the container exits as well. 
By default the `ubuntu` image runs `bash` and as you don't pass any commands to it, 
it exits immediately. This is no fun, though.

Let's try to keep this container alive for some time by using the `-it` flags. 
This tells Docker to create an interactive session.

    docker run -it ubuntu

Now you should see something like:

    root@73335ebd3355:/#

Keep in mind that the part after `@` will be different --- that is your container ID.

In this manual, we will use the following icon to show that the command should be run 
in the container:

    command to be run inside the container

You are now in your new `ubuntu` container! 
Try to play around, you can try to use some basic `bash` commands like `ls`, `cd`, `cat` 
to make sure that you are not in your host machine.

You can check which containers you are running using the `docker ps` command --- 
analogous to the `ps` command. 
Open a new terminal window (don't close the other one yet) and type:

    docker ps

An alternative (more explicit) syntax is

    docker container list

These commands list all running containers.

Now you can go back to your `ubuntu` container and type `exit`. 
This will bring you back to you host shell and will stop the container. 
If you again run the `docker ps` command you will see nothing running. 
So does this mean that this container and all changes you might have made in it are gone? 
Not at all, `docker ps` and `docker container list` only list the *currently running* 
containers.

You can see all containers, including the stopped ones with:

    docker container list -a

Here `-a` stands for *all*. 
You will see you have two `ubuntu` containers here. 
There are two containers because every time you use `docker run`, 
a new container is created. Note that their names seem strangely random. 
We could have added custom, more descriptive names---more on this later.

We don't really need these containers, so let's get rid of them:

    docker container rm ![container name 1] ![container name 2]

You need to put your container names after `rm`. 
Using the containr IDs instead is also possible. 
Note that if the container you are trying to remove is still running you will 
be asked to stop it first.

You might need to do some other operations with containers. 
For example, sometimes you want to start or stop an existing container.
You can simply do that with:

    docker container start ![container name]
    docker container stop ![container name]
    docker container restart ![container name]

Imagine you are running a container in the background. 
The main process is running but you have no shell attached. 
How can you interact with the container? 
You can open a terminal in the container with:

    docker attach ![container name]


(basic-docker-running-options)=
## Running images

Often we will ask you to run containers with more sophisticated 
options than what we saw before. 
Look at the following example: (don't try to run this, it will not do much).

    docker -H hostname.local run -dit --privileged --name joystick --network=host -v /data:/data duckietown/rpi-duckiebot-joystick-demo:master18

[](#tab:docker-run-tab) shows a summary of the options we use most often in Duckietown. 
Below, we give some examples


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
  - Allocate a pseudo-TTY, gives you terminal access to the container, typically used together with `-i`.
* - `-d`
  - `--detach`
  - Run container in background and print container ID.
* -
  - `--name`
  - Sets a name for the container. If you don't specify one, a random name will be generated.
* - `-v`
  - `--volume`
  - Bind mount a volume, exposes a folder on your host as a folder in your container. Be very careful when using this.
* - `-p`
  - `--publish`
  - Publish a container's port(s) to the host, necessary when you need a port to communicate with a program in your container.
* - `-d`
  - `--device`
  - Similar to `-v` but for devices. This grants the container access to a device you specify. Be very careful when using this.
* -
  - `--privileged`
  - Give extended privileges to this container. That includes access to **all** devices. Be **extremely** careful when using this.
* -
  - `--rm`
  - Automatically remove the container when it exits.
* - `-H`
  - `--hostname`
  - Specifies remote host name, for example when you want to execute the command on your Duckiebot, not on your computer.
* -
  - `--help`
  - Prints information about these and other options.

:::


### Examples

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


## Other useful commands

### Pruning images

Sometimes your docker system will be clogged with images, containers, networks, etc. 
You can use `docker system prune` to clean it up.

    docker system prune

Keep in mind that this command will delete **all** containers that are not currently 
running and **all** images not used by running containers. 
So be extremely careful when using it.


### Portainer

Often, for simple operations and basic commands, one can use Portainer.

Portainer is itself a Docker container that allows you to control the Docker 
daemon through your web browser. You can install it by running:

    docker volume create portainer_data
    docker run -d -p 9000:9000 --name portainer --restart always -v /var/run/docker.sock:/var/run/docker.sock -v portainer_data:/data portainer/portainer

Note that Portainer comes pre-installed on your Duckiebot, 
so you don't need to run the above command to access the images and containers 
on your robot. You still might want to set it up for your computer.


## Hands on

Before you can do any software development in Duckietown, you need to get
comfortable with Docker and its tools.

Complete the following steps before proceeding to the next section:

1. [Install Docker](https://docs.docker.com/get-docker/)
2. [Orientation and Setup](https://docs.docker.com/get-started/)
3. [Build and run your image](https://docs.docker.com/get-started/part2/)
4. [Share images on Docker Hub](https://docs.docker.com/get-started/part3/)

If you still feel like there is something that you are missing about Docker, 
you might want to spend some time going through 
[this guide](https://docker-curriculum.com/) 
as well.


## Ask the community

If you need help with Docker basics or the use of Docker in Duckietown,
join the Slack channel [#help-docker](https://duckietown.slack.com/archives/C89SNCY4Q).
