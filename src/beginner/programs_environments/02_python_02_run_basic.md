(program-basics-run)=
## Run a basic program on your Laptop

Now that we know how to build a Docker image for Duckietown, let's put some code in one of them.

We will see how to write a simple Python program, but any language should do it.

Open a terminal and go to the directory `my-program` created above. In Duckietown, Python code must belong to a Python package. Python packages are placed inside the directory code in `my-program`. Let go ahead and create a directory called `my_package` inside packages.

    laptop $ mkdir -p ./packages/my_package

A Python package is simply a directory containing a special file called `__init__.py`. So, let’s turn that `my_package` into a Python package.

    laptop $ touch ./packages/my_package/__init__.py

Now that we have a Python package, we can create a Python script in it. Use your favorite text editor to create the file `./packages/my_package/my_script.py` and place the following code inside it.

```python
message = "Hello World!"
print(message)
```

We now need to tell Docker we want this script to be the one executed when we run the command `docker run`. In order to do so, open the file `./launchers/default.sh` and replace the line

``` 
echo "This is an empty launch script. Update it to launch your application."
```

with the line

``` 
dt-exec python3 -m "my_package.my_script"
```

Note: Always prepend `dt-exec` to the main command in `./launchers/default.sh`.

If you are curious about why that is important, we can tell you that it helps us deal with an interesting problem
 called "The zombie reaping problem" (more about this in this [article][article]).

[article]: https://blog.phusion.nl/2015/01/20/docker-and-the-pid-1-zombie-reaping-problem/
 
 You can also create different custom executable scripts, if you want to know more about that check out the file ./readme.md.    
 
Let us now re-build the image:

    laptop $ dts devel build -f 

and run it:

    laptop $ dts devel run

This will show the following message:

```
The environment variable VEHICLE_NAME is not set. Using '774a2521b42e'.
Adding /code/my-program to PYTHONPATH
Adding /code/dt-commons to PYTHONPATH
Activating services broadcast...
Done!

Hello World!

Deactivating services broadcast...
Done!
```


Congratulations! You just built and run your own Duckietown-compliant Docker image.

