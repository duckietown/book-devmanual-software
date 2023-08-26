(dtproject)=
# Beginner - The **DTProject**

```{needget}
* [Working environment setup](book-opmanual-duckiebot:laptop-setup)
* [Accounts setup](book-opmanual-duckiebot:dt-account)
* Basic knowledge of Python
---
* Learn how to use DTProjects, the most important building block in Duckietown
```


## Table of contents

```{tableofcontents}
```

---

In Duckietown, everything runs in Docker containers.
All you need in order to run a piece of software in Duckietown is a Duckietown-compliant Docker image
with your software in it.

Duckietown-compliant Docker images are built out of Duckietown Projects, in short `DTProjects`.
A DTProject is a git repository with an agreed-upon structure that our automated tools can parse.
Everybody can use existing DTProjects and everybody can create new ones and distribute them freely 
over the internet. Agreeing on a structure for our code is crucial for the creation of a community 
of developers who can easily share their solutions.

High level robot behaviors in Duckietown, such as autonomous driving (in Duckiebots) or autonomous 
flight (in Duckiedrones), are implemented collectively by a set of DTProjects.
Breaking down a complex problem into smaller problems that are tackled independently is very common 
in software development and is inspired by a military strategy called "divide and conquer" 
(latin: _divide et impera_) commonly used by the Roman Empire.

By implementing complex behaviors as the union of smaller and simpler DTProjects, we can drastically 
improve the **usability**, **mantainability**, and **modularity** of our software solutions.
Moreover, DTProjects can be exchanged, shared, extended, and improved with and by the community.


## Each DTProject produces a Docker Image

As we will see in the next sections, a DTProject needs to be built into its corresponding 
executable Docker image before it can be executed.

Understanding the pros and cons of forcing code to only run inside Docker containers right 
now is crucial.

_Bad news first!_ The biggest downside of using Docker to isolate the execution of our code
is that by doing so, we are wrapping our source code inside a Docker image. This
makes it harder for us to do development, since our code will not be easily accessible
through our local file system. This is what scares/frustrates people away from Docker
the most. Keep it in mind, if it happens to you, you are not the only one.
The Duckietown development workflow explained in this book aims, among other things,
at reducing the effect of this code isolation. We will get back to this topic later
in the book.

_As for the good news_, i.e., why using Docker to isolate our code makes sense and our
life easier, we could write a book about it, but it is better if we discover those benefits as we
go.


## Structure of a DTProject

DTProjects have an agreed-upon files structure with known locations for source code, 
configuration files, dependencies lists, etc. 

### Meta-files

* `.dtproject`: Signals that this directory contains a Duckietown Project;
* `.gitignore`: List of files and directories ignore by `git`;
* `.dockerignore`: List of files and directories ignore by `docker`;
* `.bumpversion.cfg`: Configuration for bumpversion, used to perform semantic versioning on the project;

### Docker

* `Dockerfile`: Dockerfile building the project's Docker image;
* `configurations.yaml`: Collection of Docker container configurations that can be used on this project;


### Source code

* `packages/`: This directory can contain both Python and Catkin packages;
* `launchers/`: Scripts that can be used as entry scripts inside the project's Docker container; 


### Dependencies

* `dependencies-apt.txt`: List of dependency packages that can be installed via `apt`;
* `dependencies-py3.dt.txt`: List of Duckietown-owned dependency packages that can be installed via `pip`;
* `dependencies-py3.txt`: List of third-party dependency packages that can be installed via `pip`;


### Assets and Documentation

* `assets/`: Store static assets in this directory. For example, configuration files you need to bake into the image;
* `docs/`: Contains a book project that can be used to write documentation about this project;
* `html/`: Hosts the HTML of the compiled documentation in `docs/`;


### Other

* `LICENSE.pdf`: The Duckietown Software Terms of Use;
* `README.md`: Brief description of the DTProject;


## Project Templates

While a DTProject exposes a lot of parameters that the final user can tune, e.g., base Docker image,
support for ROS, etc., it helps to have a set of predefined presets covering the most common use cases.
We call these **Project Templates**. Project templates are stored on GitHub as **Template Repositories**. 
These are of a special kind of repositories and their main characteristic is that one can use them to 
initialize new repositories.

The simplest module template is called `basic` and its template
repository is [duckietown/template-basic](https://github.com/duckietown/template-basic).
Since understanding the differences between different templates is outside the scope of
this section, we can use any template for the remainder of this section, we suggest
using the one above.
For a list of predefined project templates, check out the [](project-templates)

In the next sections of this chapter we will learn how to customize, build, and run our own DTProjects
both locally and on a Duckietown robot.
