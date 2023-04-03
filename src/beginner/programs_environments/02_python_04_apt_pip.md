(program-basics-dep-install)=
## Install dependencies using package managers (e.g., `apt`, `pip`)

It is quite common that our programs need to import libraries, thus we need a way to install them. Since our programs reside in Docker images, we need a way to install libraries in the same image. 

The template provided by Duckietown supports two package managers out of the box:

- Advanced Package Tool (`apt`)
- Pip Installs Packages for Python3 (`pip3`)

List your apt packages or pip3 packages in the files `dependencies-apt.txt` 
and `dependencies-py3.txt` respectively before running the command `dts devel build`.

(exercise:ex-docker-numpy)=
#### Basic NumPy program

Write a program that performs the sum of two numbers using [NumPy](https://numpy.org/). 
Add `numpy` to the file `dependencies-py3.txt` to have it installed in the Docker image.


Here you go! Now you can handle pip dependencies as well!
