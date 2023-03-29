# Beginner - Software Modules

# Modules {#part:devel_modules level=part status=ready}

Author: Andrea F. Daniele

Maintainer: Andrea F. Daniele

This section of the book focuses on the concept of software module in Duckietown.

Devices in Duckietown (e.g., Duckiebots, Watchtowers, etc) are not configured to accept
code directly. The Operating System running on the on-board computers is configured
to accept only code running inside Docker containers. Modules are an easy, robust and 
effective way of wrapping code into executable Docker images.

Remember, you are not allowed to run any code on any of these devices outside a proper 
Duckietown module. So, if you have code to run, you need to put it in a module first.

# Introduction {#sec:devel_modules_intro level=sec status=ready}

Author: Andrea F. Daniele

Maintainer: Andrea F. Daniele




A software module in Duckietown implements a high level behavior, for example,
autonomous driving in Duckietown. A software module is broken into a set of smaller
pieces, called nodes. This allows us to tackle a complex problem by leveraging solutions
to smaller problems. This approach is very common in software development and is
inspired by a military strategy called "divide and conquer" (latin: _divide et impera_)
commonly used by the Roman Empire.   
Breaking a module into nodes is not trivial, and where you draw the lines between nodes
can make a huge difference in the final outcome. Some qualities of the final software
product directly affected by this decision are **usability**, **mantainability**,
and **portability**.

Note: Read the [ISO/IEC 9126](#sec:developer_basics_iso_9126) standard
to learn more about product quality in software engineering. If you want to become
a developer, you might want to bookmark that URL, you will need it, **a lot**. 



<minitoc/>


## What is a module



A module in Duckietown is a Docker image that complies with a **module template**
(more about templates in the next section). Remember the 
[ISO/IEC 9126](#sec:developer_basics_iso_9126) standard? well, 
modules are designed to be highly **portable** and  **usable**.


Modules have a pre-defined file system structure with fixed locations for source code 
and configuration files. File system structure and default locations are 
template-dependent, check the section **TBD** to understand the
different module templates available in Duckietown.


Understanding the pros and cons of using Docker to isolate modules right now is crucial.



Bad news first! The biggest negative effect of using Docker to isolate modules
is that by doing so, we are wrapping our source code inside a Docker image. This
makes it harder for us to do development, since our code will not be easily accessible
through our local file system. This is what scares/frustrates people away from Docker 
the most. Keep it in mind, if it happens to you, you are not the only one.
The Duckietown development workflow explained in this book aims, among other things,
at reducing the effect of this code isolation. We will get back to this topic later
in the book.

As for the good news, i.e., why using Docker to isolate modules makes sense and our
life easier, we could write a book about it, but they will become clear as we proceed.



Duckietown defines a set of module types that you can choose from. The list of module
types and their differences will be the topic of the section  **TBD**. What is important to know for now, is that a module
type defines the environment your code will run in. For each module type, a template
repository is provided.


## Module Templates

A module template is a repository, hosted on GitHub space of the Duckietown organization,
that lets you build a new module of that type.

Templated repositories on GitHub are special repositories that you can use to initialize
an empty repository. Templated repository will provide an initial structure to your empty
repository.

Although different template repositories have different files and structures, they all 
contain: 
a `Dockerfile`, used to _compile_ the entire repository into a Docker image;
one or more requirements files, in which you can list the dependencies of your code; 
a `.dtproject` file that makes it compatible with the duckietown shell.
You will find other files as well, but these are the most important ones.

The simplest module template is called `basic` and its template 
repository is [duckietown/template-basic](https://github.com/duckietown/template-basic).
Since understanding the differences between different templates is outside the scope of 
this section, we can use any template for the remainder of this section, we suggest 
using the one above.



## Create your own module {#devel_create_module_repo}



In order to be able to create a Duckietown module, you need to gain access to
the module template repositories on GitHub.
There are two way to achieve this: you are an official Duckietown developer, thus
you are part of the Duckietown organization on GitHub; or, you create a copy (fork) 
of the template you need on your GitHub account.

If you are not a member of the Duckietown organization on GitHub, you can fork
a template on your GitHub account by visiting the module template page on GitHub
(e.g., [duckietown/template-basic](https://github.com/duckietown/template-basic))
and click on the Fork button at the top-right corner of the page.


<figure>
    <figcaption>Fork button on GitHub</figcaption>
    <img alt="github.com fork button" style='width:26em' src="images/github_fork_button.jpg"/>
</figure>




Once you gained access to the template (either by joining the Duckietown developers
team or forking the template repository), you are able to create a new repository that
will store your module based on the template repository.
To do so, go to GitHub, click on the [+] button on the header at the top-right corner
and then choose **New repository**. In the _New repository_ page, choose the template
(e.g., `duckietown/template-basic`) and enter the name of your new module 
(e.g., `my_module`) as shown in the image below.


<figure>
    <figcaption>New repository with template on GitHub</figcaption>
    <img alt="new templated repository on github" style='width:32em' src="images/github_new_repo_w_template.jpg"/>
</figure>

<br/>

Click on **Create repository** to create the module repository.

## Build a module

Building a module is very simple. To start, open a terminal and clone a module repository 
(we created one in section **TODO** ).

Templates leave placeholders that you will need to replace with the proper information
about your module before you can build it.

Open the file `Dockerfile` using any text editor and look for the following lines
at the top of the file:

**TODO**

<!---
```
dockerfile
ARG REPO_NAME="<REPO_NAME_HERE>"
ARG DESCRIPTION="<DESCRIPTION_HERE>"
ARG MAINTAINER="<YOUR_FULL_NAME> (<YOUR_EMAIL_ADDRESS>)"
```
-->

Replace the placeholders strings with, respectively,
 - the name of the repository (i.e., `my_module`); 
 - a brief description of the functionalities of the module
 - your name and email address to claim the role of maintainer; 
 
Save and return to the terminal. Now run the following command to build the module.

```
dts devel build -f
```

The flag `-f` (short for `--force`) is needed in order to allow `dts` to build a module
out of a non-clean repository. A repository is not clean when there are changes that are
not committed (and in fact our change to `Dockerfile` is not). 
This check is in place to prevent developers from forgetting to push local changes.
If the build is successful, you will see something like the following.

<figure>
    <figcaption>Result of command `dts devel run`</figcaption>
    <img alt="dts devel build" style='width:36em' src="images/dts_devel_build_ex1.jpg"/>
</figure>

<br/>

Congrats! You just built your first Duckietown-compatible software module.



## Run a module

As stated above, building a module produces a Docker image. This image is the
_compiled_ version of your source project. You can find the name of the resulting 
image at the end of the output of the `dts devel build` command.
In the example above, look for the line

```
Final image name: duckietown/my_module:v1-amd64
```




## Hands on

TODO: This is a templated subsection


## Ask the community

TODO: This is a templated subsection

# Module Types {#sec:devel_module_types level=sec status=ready}

Author: Andrea F. Daniele

Maintainer: Andrea F. Daniele


In Duckietown, there are _three_ main module types.
For each module type we provide a repository template. 
The module types are:

- **basic** (template: [duckietown/template-basic](https://github.com/duckietown/template-basic))
- **ros** (template: [duckietown/template-ros](https://github.com/duckietown/template-ros))
- **core** (template: [duckietown/template-core](https://github.com/duckietown/template-core))

We will go through each one of them in details in the next sections.


## Module Type: Basic

The module type **basic** is the one
