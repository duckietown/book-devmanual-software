(python-programs-environments)=
# Beginner - Programs and Environments

<!--
Excerpt: Learn how to setup a Python project and associated tools.
-->

```{needget}
* [Laptop setup](+opmanual_duckiebot#laptop-setup)
* Basic knowledge of Python
---
* Developer knowledge of Python
```

We assume you are already quite comfortable with Python. Nevertheless, when you work with big and complex projects, there are some subtleties that you must consider and some handy tools that can make your life easier. Let's take a look at some of these now.

(python-project-basics)=
## Define a basic project structure  

In Duckietown, everything runs in Docker containers. All you need in order to run a piece of software in Duckietown is a Duckietown-compliant Docker image with your software in it.

A boilerplate is provided by the following [Duckietown repository](https://github.com/duckietown/template-basic/).

The repository contains a lot of files, but do not worry, we will analyze them one by one. Click on the green button that says "Use this template". 

```{figure} ../../_images/programs_environment/use_this_template.png
:width: 400px
:name: fig:prog-env-template

Use this template.
```

<!--
<figure>
  <img style="width:40em" src="images/use_this_template.png"/>
</figure>
-->

This will take you to a page that looks like the following:

```{figure} ../../_images/programs_environment/create_a_repo_2.png
:width: 500px
:name: fig:prog-env-create-repo

Creating a repository.
```

<!--
<figure>
  <img style="width:40em" src="images/create_a_repo_2.png"/>
</figure>
-->


Pick a name for your repository (say `my-program`) and press the button *Create repository from template*. Note, you can replace `my-program` with the name of the repository that you prefer, make sure you use the right name in the instructions below.

This will create a new repository and copy everything from the repository `template-basic` to your new repository. You can now open a terminal and clone your newly created repository.

    laptop $ git clone https://github.com/![YOUR_NAME]/my-program
    laptop $ cd my-program

Note: Replace `YOUR_NAME` in the link above with your GitHub username.

The repository contains already everything you need to create a Duckietown-compliant Docker image for your program. The only thing we need to change before we can build an image from this repository is the repository name in the file Dockerfile. Open it using the text editor you prefer and change the first line from:

```Dockerfile
ARG REPO_NAME="![REPO_NAME_HERE]"
```

to

```Dockerfile
ARG REPO_NAME="my-program"
```

and then similarly update the `DESCRIPTION` and the `MAINTAINER` ARGs. 

Save the changes. We can now build the image, even though there is not going to be much going on inside it until we place our code in it. 

Now, in a terminal, move to the directory created by the `git clone` instruction above and run the following command(beware that it might take some time):

    laptop $ dts devel build -f

Note: If the above command is not recognized, you will first have to install it with `dts install devel`.


If you correctly installed Docker and `dts`, you should see a long log that ends with something like (but not necessary exactly like) the following:

```{figure} ../../_images/programs_environment/dts_devel_build.png
:width: 500px
:name: fig:build

Building a container through the development command in the Duckietown shell.
```

<!--
<figure id="fig:build">
  <img alt="build" style="width:40em" src="images/dts_devel_build.png"/>
</figure>
-->

You can now run your container by executing the following command.

    laptop $ dts devel run

This will show the following message:

```
The environment variable VEHICLE_NAME is not set. Using '![LAPTOP_HOSTNAME]'.
WARNING: robot_type file does not exist. Using 'duckiebot' as default type.
WARNING: robot_configuration file does not exist.
=&gt; Launching app...
This is an empty launch script. Update it to launch your application.
&lt;= App terminated!
```

Congratulations! You just built and run your first Duckietown-compliant Docker image.


