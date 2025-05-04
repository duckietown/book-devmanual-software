```{seo}
:description: Learn how to add your own Python code to a Duckietown DTProject and build a compliant Docker image using the Duckietown development workflow.
:keywords: Duckietown, DTProject, Python, Docker, development, robot software, robotics, dts, docker build
```


(dtproject-add-your-code)=
# Add your code

Now that we know how to build Docker images for Duckietown, let us build one with a simple Python program inside.

Open a terminal and go to the directory `my-project` created in the previous page. In Duckietown, Python code must belong to a Python package. Python packages are placed inside the directory `packages/` found at the root of `my-project`. Go ahead and create a directory called `my_package` inside `packages/`.

    mkdir -p ./packages/my_package

A Python package is defined by the presence of a special initialization file named `__init__.py`. Create it as follows:

    touch ./packages/my_package/__init__.py

With the package directory in place, add a Python script to it. Using a text editor or integrated development environment (IDE), create the file `./packages/my_package/my_script.py` with the following content:

```python
message = "\nHello World!\n"
print(message)
```

Next, configure the project so that this script is executed by default when running the container. Open the file `./launchers/default.sh` and replace the placeholder line:

```bash
echo "This is an empty launch script. Update it to launch your application."
```

with:

```bash
dt-exec python3 -m "my_package.my_script"
```


```{note}
Always prepend `dt-exec` to the command in `./launchers/default.sh`. 

This utility ensures proper signal handling and process cleanup inside the container ("[The zombie reaping problem](https://blog.phusion.nl/2015/01/20/docker-and-the-pid-1-zombie-reaping-problem/)").
```

For information on defining additional launcher scripts, refer to [](dtproject-launchers).

Now rebuild the Docker image with:

    dts devel build -f 

and run it:

    dts devel run

The expected output will be:

```
...
==> Launching app...

Hello World!

<== App terminated!
```

```{admonition} Congratulations 🎉
You just built and ran your own Duckietown-compliant Docker image with custom Python code inside!
```
