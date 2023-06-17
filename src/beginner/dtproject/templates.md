(project-templates)=
# Project Templates

While DTProjects are open to all sorts of customizations to accommodate for virtually any need, 
Duckietown provides a set of templated projects that cover the most common use cases.
For each project template we provide a templated repository.


## Coding project templates

These project templates are designed for projects implementing robot behaviors (e.g., lane following),
back-end systems (e.g., REST APIs), etc.

```{list-table} Coding project templates
:header-rows: 1
:name: project-templates-code

* - Name
  - Features
  - Link
* - **basic**
  - - Ubuntu 20.04 base image
    - Support for Python packages in `packages/`
  - [duckietown/template-basic](https://github.com/duckietown/template-basic)
* - **ros**
  - - Same as **basic**
    - Support for ROS
    - Support for catkin packages in `packages/`
  - [duckietown/template-ros](https://github.com/duckietown/template-ros)
* - **core**
  - - Same as **ros**
    - Duckietown autonomous driving modules baked in
  - [duckietown/template-core](https://github.com/duckietown/template-core)
```


## Documentation project templates

These project templates are designed for projects implementing documentation books and manuals such as
the one you are looking at right now.

```{list-table} Documentation project templates
:header-rows: 1
:name: project-templates-docs

* - Name
  - Features
  - Link
* - **book**
  - - Based on [Jupyter Book](https://jupyterbook.org/en/stable/intro.html)
    - Compiles into HTML and PDF
    - Easy cross-reference with Duckietown books library
  - [duckietown/template-book](https://github.com/duckietown/template-book)
```


## Duckietown Learning Experiences (LXs) project templates 

These project templates are designed for the development of web-based dashboards based on `\compose\`. 
A project based on this template is the robot dashboard.

```{list-table} Duckietown Learning Experiences (LXs) project templates 
:header-rows: 1
:name: project-templates-lx

* - Name
  - Link
* - **lx**
  - [duckietown/template-lx](https://github.com/duckietown/template-lx)
* - **lx-recipe**
  - [duckietown/template-lx-recipe](https://github.com/duckietown/template-lx-recipe)
```


## Dashboard project templates

These project templates are designed for the development of web-based dashboards based on `\compose\`.
A project based on this template is the robot dashboard.

```{list-table} Dashboard project templates
:header-rows: 1
:name: project-templates-dashboards

* - Name
  - Link
* - **compose**
  - [duckietown/template-compose](https://github.com/duckietown/template-compose)
```
