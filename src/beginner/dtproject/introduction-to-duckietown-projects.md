```{seo}
:description: Learn what a Duckietown Project (DTProject) is and how it structures and containerizes robotics software using Docker for modular development.
:keywords: Duckietown, DTProject, Docker, robotics software, modular code, ROS, project structure, containerization
```

(dtproject)=
# Beginner - The `DTProject`

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
(dtproject-intro)=
## Introduction

In Duckietown, all code runs inside Docker containers.

To run software in Duckietown you will need:

* a Duckietown-compliant Docker image, and 
* "payload" software inside it.

Duckietown-compliant Docker images are built out of Duckietown Projects, in short, `DTProjects`.

A DTProject is a Git repository with an agreed-upon structure that the Duckietown automated tools can parse. Having a standardized format is good for reproducibility of outcomes and development speed, making it possible to exchange, share, extend, and improve each other's projects with relative ease.

A set of DTProjects can, for example, implement high-level robot behaviors, such as autonomous driving (in Duckiebots) or autonomous flight (in Duckiedrones).

Breaking down a complex problem into smaller ones that are tackled independently is common in software development and not only. The ancient Romans formalized this in the context of military strategy with the well-known "divide and conquer" (in latin: _divide et impera_).

By implementing complex behaviors as the union of smaller and simpler DTProjects, we improve the **usability**, **mantainability**, and **modularity** of the software.

(dtproject-docker-image)=
## Each DTProject produces a Docker Image

A DTProject needs to be built into its corresponding executable Docker image before it can be run.

Forcing code to only run inside Docker containers has advantages and disadvantages:

_The bad news:_ The biggest downside of using containers to isolate the execution of Duckietown code is that by doing so the source code is wrapped inside an extra layer of code, the Docker image. 

This _isolates_ the code, making it harder to do development because it will not be directly accessible through the local file system. This can be frustrating at times.

_The good news:_ the main advantage of a fully Dockerized software architecture is the reproducibility of outcomes. Additional advantages will become evident hereafter. 

(dtproject-structure)=
## Structure of a DTProject

DTProjects standardize locations for source code, configuration files, dependencies lists, and more. 

(dtproject-meta-files)=
### DTProject meta-files

* `.dtproject`: indicates that this directory contains a Duckietown Project;
* `.gitignore`: lists files and directories ignored by `git`;
* `.dockerignore`: lists files and directories ignored by `docker`;
* `.bumpversion.cfg`: configuration for [`bumpversion`](https://pypi.org/project/bumpversion/), used to perform semantic versioning on the project;

(dtproject-docker-files)=
### Docker

* `Dockerfile`: Dockerfile building the project's Docker image;
* `configurations.yaml`: collection of Docker container configurations that can be used on this project;

(dtproject-source-code)=
### Source code

* `packages/`: this directory can contain both Python and Catkin packages;
* `launchers/`: code that can be used as entry scripts inside the project's Docker container; 

(dtproject-dependencies)=
### Dependencies

* `dependencies-apt.txt`: List of dependency packages that can be installed via `apt`;
* `dependencies-py3.dt.txt`: List of Duckietown-owned dependency packages that can be installed via `pip`;
* `dependencies-py3.txt`: List of third-party dependency packages that can be installed via `pip`;

(dtproject-assets-docs)=
### Assets and Documentation

* `assets/`: store static assets in this directory. For example, configuration files that are needed in the image;
* `docs/`: contains a book project used to create documentation (like this book you are reading now);
* `html/`: hosts the `html` version of the compiled documentation in `docs/`;

(dtproject-other-files)=
### Other

* `LICENSE.pdf`: The Duckietown Software Terms of Use; Note that you must agree to its terms to use DTprojects. 
* `README.md`: Brief description of the DTProject;

(dtproject-templates)=
## Project Templates

DTProjects expose many tunable parameters, e.g., for the base Docker image, or ROS support, and offer presets (_project templates_) for the most common use cases. Project templates are stored on GitHub as _template repositories_. 

Template repositories are special, in that their main purpose is to initialize new repositories.

The simplest template is called `basic`, and its template
repository is [duckietown/template-basic](https://github.com/duckietown/template-basic).

For a list of predefined project templates, check out the [](project-templates)

In the next sections we will show how to customize, build, and run DTProjects; both locally and on a Duckietown robot.