(program-basics-dep-install)=
# Define dependencies

It is quite common that our programs need to import libraries, thus we need a way to install them. 
Since our programs reside in Docker images, we need a way to install libraries inside the image when the
image is built. 

All the project templates provided by Duckietown support two package managers out of the box:

- Advanced Package Tool (`apt`)
- Pip Installs Packages for Python3 (`pip3`)

List your apt packages or pip3 packages in the files `dependencies-apt.txt`.
As for `pip3` dependencies, we make a distinction between Duckietown-owned and third-party libraries.
List all the Duckietown-owned libraries you want to install in the file `dependencies-py3.dt.txt` and
third-party libraries in the file `dependencies-py3.txt`.

```{note}
Dependencies files support comments (lines starting with `#`) and empty lines. Use them to group
dependencies together and make dependencies lists easier to read and maintain.
```

Running `dts devel build` after editing these files will rebuild the image with the new dependencies
installed.

**That's it!** Now you know how to customize dependencies as well! 


(exercise:dtproject-deps-numpy)=
## Exercise: Basic NumPy program

Write a program that performs the sum of two numbers using [NumPy](https://numpy.org/). 
Add `numpy` to the file `dependencies-py3.txt` to have it installed in the Docker image.
