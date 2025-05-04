```{seo}
:description: Learn how to define and install external dependencies in Duckietown DTProjects using apt and pip, and try a NumPy coding exercise.
:keywords: Duckietown, DTProject, dependencies, Docker, Python, pip, apt, NumPy, robotics development
```

(program-basics-dep-install)=
# Defining `DTproject` dependencies

It is common for robotics software to require additional libraries, which must be made available inside the Docker image during the build process.

All Duckietown project templates support dependency installation using two package managers:

- Advanced Package Tool: **APT** (`apt`), for system-level packages
- Pip Installs Packages for Python3 (`pip3`) for Python packages

Specify dependencies in the following files:

* `dependencies-apt.txt`: system packages installed via APT

* `dependencies-py3.txt`: third-party Python packages from PyPI

* `dependencies-py3.dt.txt`: Duckietown-maintained Python packages


```{note}
These dependency files support comments (lines beginning with `#`) and blank lines. Use these features to group packages logically and maintain readability.
```

After updating any of these files, rebuild the Docker image with:

    dts devel build

This ensures that the specified packages are installed and available inside the containerized development environment.


(exercise:dtproject-deps-numpy)=
## Exercise: Basic NumPy program

Write a program that performs the sum of two numbers using [NumPy](https://numpy.org/). 
Add `numpy` to the file `dependencies-py3.txt` to have it installed in the Docker image.
