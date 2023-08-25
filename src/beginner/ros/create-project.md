(ros-project-create-new)=
# New ROS DTProject

```{needget}
* [Working environment setup](book-opmanual-duckiebot:laptop-setup)
* [Accounts setup](book-opmanual-duckiebot:dt-account)
---
* Learn how to create a new ROS-compatible DTProject from a template
```


## Create project from a template

Visit the template repository page
[duckietown/template-ros](https://github.com/duckietown/template-ros/).
Click on the button that reads "Use this template" and then choose 
"Create a new repository" from the dropdown menu.

```{figure} ../../_images/beginner/github_use_template.jpg
:width: 60%
:name: fig:ros-github-use-template

Use template repository on GitHub.
```


This will take you to a page that looks like the following:

```{figure} ../../_images/beginner/ros/create-repo-from-template.png
:width: 90%
:name: fig:ros-create-repo-from-template

Creating a repository from template.
```

Make sure the selected template is `duckietown/template-ros`, pick a name for 
our repository (e.g., `my-ros-project`), and add a description for the new repository.

Click on the button *Create repository*.

```{note}
You can replace `my-ros-project` with the name of the repository that you prefer. 
If you do change it, make sure you use the right name in the instructions going forward.
```

This will create a new repository starting from the content of the template `template-ros`.
You can now open a terminal and clone your newly created repository.

    git clone https://github.com/YOUR_NAME/my-ros-project
    cd my-ros-project

```{note}
Replace `YOUR_NAME` in the link above with your GitHub username.
```


## Edit placeholders

As we learned in [](dtproject-edit-placeholders), we now need to edit the placeholders left by the 
template. Head over to the `Dockerfile` and edit the following lines using any text editor:

```Dockerfile
ARG REPO_NAME="<REPO_NAME_HERE>"
ARG DESCRIPTION="<DESCRIPTION_HERE>"
ARG MAINTAINER="<YOUR_FULL_NAME> (<YOUR_EMAIL_ADDRESS>)"
```

Replace the placeholders strings with, respectively,

- the name of the repository (i.e., `my-ros-project`);
- a brief description of the functionalities implemented in this project;
- your name and email address to claim the role of maintainer;


Save the changes. We can now compile this project into a Docker image.


## Build the project

Exactly as in [](dtproject-build-project), we can now move to the project root and run the 
following command to build our project (beware that it might take some time):

    dts devel build -f

Again, building a project produces a Docker image. This image is the
_compiled_ version of your source project. You can find the name of the resulting
image at the end of the output of the `dts devel build` command.
In this case, look for the line:

```sh
Final image name: duckietown/my-ros-project:v2-amd64
```


## Run the project

Again, as in [](dtproject-run-project), we can run our project by executing the command,

    dts devel run

This will show the following message:

```
...
==> Launching app...
This is an empty launch script. Update it to launch your application.
<== App terminated!
```


```{admonition} Congratulations 🎉
You just built and run your first ROS-based Duckietown-compliant Docker image.
```