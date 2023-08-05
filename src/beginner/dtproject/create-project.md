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

Duckietown-compliant Docker images are built out of Duckietown Projects, in short `DTProjects`.
A boilerplate for the simplest DTProject is provided by the 
[duckietown/template-basic](https://github.com/duckietown/template-basic/) 
repository.


## Create project from a template

Visit the template repository page
[duckietown/template-basic](https://github.com/duckietown/template-basic/).
Click on the button that reads "Use this template" and then choose 
"Create a new repository" from the dropdown menu.

```{figure} ../../_images/beginner/github_use_template.jpg
:width: 70%
:name: fig:basic-github-use-template

Use template repository on GitHub.
```

This will take you to a page that looks like the following:

```{figure} ../../_images/beginner/basic/create-repo-from-template.png
:width: 90%
:name: fig:basic-create-repo-from-template

Creating a repository from template.
```

Pick a name for your repository (say `my-program`) and press the button *Create repository from template*. 
Note, you can replace `my-program` with the name of the repository that you prefer, if you do change it,
make sure you use the right name in the instructions going forward.

This will create a new repository and copy everything from the repository `template-basic` to your 
new repository. You can now open a terminal and clone your newly created repository.

    git clone https://github.com/YOUR_NAME/my-program
    cd my-program

```{note}
Replace `YOUR_NAME` in the link above with your GitHub username.
```

(dtproject-edit-placeholders)=
## Edit placeholders

The repository contains already everything you need to create a Duckietown-compliant Docker image 
for your program. Before doing anything else, we need to head over to the `Dockerfile` and edit the
following lines using a text editor:

```Dockerfile
ARG REPO_NAME="<REPO_NAME_HERE>"
ARG DESCRIPTION="<DESCRIPTION_HERE>"
ARG MAINTAINER="<YOUR_FULL_NAME> (<YOUR_EMAIL_ADDRESS>)"
```

Replace the placeholders strings with, respectively,

- the name of the repository (i.e., `my-program`);
- a brief description of the functionalities of the module
- your name and email address to claim the role of maintainer;


Save the changes. We can now build the image, even though there is not going to be much going on 
inside it until we place our code in it.


(dtproject-build-project)=
## Build the project

Now, in a terminal, move to the directory created by the `git clone` instruction above and run the 
following command (beware that it might take some time):

    dts devel build -f


The flag `-f` (short for `--force`) is needed in order to allow `dts` to build a module
out of a non-clean repository. A repository is not clean when there are changes that are
not committed (and in fact our changes to `Dockerfile` are not).
This check is in place to prevent developers from forgetting to push local changes.
If the build is successful, you will see something like the following:

```{figure} ../../_images/beginner/dts_devel_build.png
:width: 100%
:name: fig:build

Building a container through the development command in the Duckietown shell.
```

As discussed above, building a project produces a Docker image. This image is the
_compiled_ version of your source project. You can find the name of the resulting
image at the end of the output of the `dts devel build` command.
In the example above, look for the line:

```sh
Final image name: duckietown/my-program:v1-amd64
```

(dtproject-run-project)=
## Run the project

You can now run your container by executing the following command.

    dts devel run

This will show the following message:

```
==> Entrypoint
   INFO: The environment variable VEHICLE_NAME is not set. Using '...'.
   WARNING: robot_type file does not exist. Using 'duckiebot' as default type.
   WARNING: robot_configuration file does not exist.
   INFO: Network configured successfully.
<== Entrypoint
==> Launching app...
This is an empty launch script. Update it to launch your application.
<== App terminated!
```

**Congratulations!** You just built and run your first Duckietown-compliant Docker image.
