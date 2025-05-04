```{seo}
:description: Create a new ROS-compatible Duckietown DTProject from template-ros, clone, configure placeholders, build and run the Docker image.
:keywords: Duckietown, ROS, DTProject, template-ros, Docker, robotics, dts, deployment
```

(ros-project-create-new)=
# New ROS DTProject

```{needget}
* [Working environment setup](book-opmanual-duckiebot:laptop-setup)
* [Accounts setup](book-opmanual-duckiebot:dt-account)
---
* Learn how to create a new ROS-compatible DTProject from a template
```


## Create a project from a template

A new repository is created from the template-ros boilerplate. Navigate to the template repository on GitHub: [duckietown/template-ros](https://github.com/duckietown/template-ros/). Select **Use this template**, then choose **Create a new repository** from the dropdown menu.


```{figure} ../../_images/beginner/github_use_template.jpg
:name: ros-github-use-template-1
:alt: GitHub Use Template button interface
:align: center
:width: 60%

Use template repository on GitHub.
```


This will take you to a page that looks like the following:

```{figure} ../../_images/beginner/ros/create-repo-from-template.png
:name: ros-create-repo-from-template-1
:alt: GitHub create repository from ROS template interface
:align: center
:width: 90%

Creating a repository from the ROS template.

```
Ensure that `duckietown/template-ros` is selected. Assign a repository name (for example, `my-ros-project`) and provide a description. Then select **Create repository**.

```{note}
Replace `my-ros-project` with the preferred repository name. If a different name is chosen, follow that name in subsequent instructions.
```

This will create a new repository starting from the content of the template `template-ros`. Clone the newly created repository:

    git clone https://github.com/YOUR_NAME/my-ros-project
    cd my-ros-project

```{note}
Replace `YOUR_NAME` in the link above with your GitHub username.
```

(ros-edit-placeholders)=
## Edit placeholders

As seen in [](dtproject-edit-placeholders), the placeholders left by the template need to be edited. Open `Dockerfile` in a text editor and update the following arguments:

```Dockerfile
ARG REPO_NAME="<REPO_NAME_HERE>"
ARG DESCRIPTION="<DESCRIPTION_HERE>"
ARG MAINTAINER="<YOUR_FULL_NAME> (<YOUR_EMAIL_ADDRESS>)"
```

Replace each placeholder as follows:

- `REPO_NAME`: the repository name (e.g., `my-ros-project`)

- `DESCRIPTION`: a concise summary of the project’s functionality

- `MAINTAINER`: full name and email address of the maintainer


Save the changes to proceed with compiling this project into a Docker image.


## Build the project

As shown in [](dtproject-build-project), navigate to the project root and run:

    dts devel build -f

Again, building a project produces a Docker image. This image is the
_compiled_ version of the source project. Upon success, the final image name is displayed, for example:

```sh
Final image name: duckietown/my-ros-project:v2-amd64
```

(ros-run-project)=
## Run the project

As shown in [](dtproject-run-project), run the project with:

    dts devel run

This will show the following message:

```
...
==> Launching app...
This is an empty launch script. Update it to launch your application.
<== App terminated!
```


```{admonition} Congratulations 🎉
You just built and ran your first ROS-based Duckietown-compliant Docker image.
```
