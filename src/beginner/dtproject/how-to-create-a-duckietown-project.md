```{seo}
:description: Learn how to create a new DTProject using the Duckietown project template and build your first Duckietown-compliant Docker image.
:keywords: Duckietown, DTProject, docker, robotics, template, GitHub, how to build a DTproject
```

(dtproject-create-new)=
# New project

```{needget}
* [Working environment setup](book-opmanual-duckiebot:laptop-setup)
* [Accounts setup](book-opmanual-duckiebot:dt-account)
* Basic knowledge of Python
---
* Learn how to use DTProjects, the most important building block in Duckietown
* Learn how to create a new DTProject from a template
```

Duckietown-compliant Docker images are built from Duckietown Projects, abbreviated as `DTProjects`. Read the [introduction to DTprojects](dtproject) if not already done.

A boilerplate for the simplest DTProject is provided by the 
[duckietown/template-basic](https://github.com/duckietown/template-basic/) 
repository.

(dtproject-template-create)=
## Create a DTproject from a template

Visit the template repository page:  
[duckietown/template-basic](https://github.com/duckietown/template-basic/).  
Select the button labeled **Use this template**, then choose **Create a new repository** from the dropdown menu.

```{figure} ../../_images/beginner/github_use_template.jpg
:name: fig-dtproject-template-button
:alt: GitHub page showing the "Use This Template" button.
:align: center
:width: 90%

Use template repository on GitHub.
```

This will redirect to a new page:

```{figure} ../../_images/beginner/basic/create_repo_from_template.png
:name: fig-dtproject-create-repo
:alt: GitHub interface to create a repository from a template.
:align: center
:width: 90%

Creating a repository from template.
```

Choose a name for the repository, for instance `my-project`, and press the **Create repository from template** button.

It is possible to replace `my-project` with any preferred name. If so, make sure to update all following references accordingly.

This creates a new repository containing a copy of all content from `template-basic`. Next, clone it locally using a terminal:

```bash
git clone https://github.com/YOUR_NAME/my-project
cd my-project
```

```{note}
Replace `YOUR_NAME` with your actual GitHub username.
```

(dtproject-edit-placeholders)=
## Edit placeholders

Before continuing, update placeholder metadata inside the `Dockerfile`. Open the file and replace the following lines:

```Dockerfile
ARG REPO_NAME="<REPO_NAME_HERE>"
ARG DESCRIPTION="<DESCRIPTION_HERE>"
ARG MAINTAINER="<YOUR_FULL_NAME> (<YOUR_EMAIL_ADDRESS>)"
```

For example:

```Dockerfile
ARG REPO_NAME="my-project"
ARG DESCRIPTION="My first Duckietown project"
ARG MAINTAINER="HappyDuckie (happyduckie@duckietown.com)"
```

Save and close the file. It is now possible to build the image, although the contents will still be minimal.

(dtproject-build-project)=
## Build the project

In a terminal, navigate to the root directory of the cloned repository and execute:

```bash
dts devel build -f
```

The `-f` flag (`--force`) allows the build to proceed even if the repository is not "clean", i.e., as after making local edits.

Upon a successful build, a Docker image will be created. Look for a message similar to the following in the terminal output:

```text
Final image name: docker.io/duckietown/my-project:v2-amd64
```

```{figure} ../../_images/beginner/basic/dts_devel_build.png
:name: fig-dtproject-dts-build
:alt: Output of building a container with Duckietown shell.
:align: center
:width: 90%

Building a container through the Duckietown development command.
```

(dtproject-run-project)=
## Run the `DTproject`

To launch the container:

```bash
dts devel run
```

This will display:

```text
...
==> Launching app...
This is an empty launch script. Update it to launch your application.
<== App terminated!
```

```{admonition} Congratulations 🎉
You have just built and run your first Duckietown-compliant Docker image.
```
