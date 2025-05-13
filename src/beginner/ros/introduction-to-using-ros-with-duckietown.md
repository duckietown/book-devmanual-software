```{seo}
:description: Learn how to set up and run ROS-based DTProjects on Duckiebot, including creating custom ROS nodes with publishers and subscribers.
:keywords: Duckietown, ROS, DTProject, Duckiebot, ROS nodes, publisher, subscriber, Docker, robotics development
```



(dtproject-ros)=
# Beginner - Use ROS

```{needget}
* A computer set up with the [Duckietown software requirements](book-opmanual-duckiebot:laptop-setup)
* An initialized [Duckiebot](book-opmanual-duckiebot:get-db-hw)
* [Completed tutorial on DTProject](dtproject)
---
* A working knowledge of ROS development in Duckietown
* A custom ROS node with a publisher and a subscriber running on your Duckiebot
```

## Table of contents

```{tableofcontents}
```

(basic-structure)=
## Basic Project Structure
In Duckietown, all software runs in Docker containers. To run ROS-based code on Duckiebot, build a Duckietown‑compliant Docker image containing the ROS software.

A [ROS DTProject template](https://github.com/duckietown/template-ros) is available on the [Duckietown GitHub](https://github.com/duckietown). If the [DTProject tutorial](dtproject) was completed, the template contents will be familiar.

As with general DTProjects, [instantiate a new repository](dtproject-create-new) from the template-ros template on GitHub. The build and execution workflow matches the DTProject tutorial.

Continue to the next sections to develop and deploy custom ROS nodes with publishers and subscribers.
