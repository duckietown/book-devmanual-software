(dtproject-add-your-code)=
# Add your code

Now that we know how to build Docker images for Duckietown, 
let's build one with a simple Python program inside.

Open a terminal and go to the directory `my-project` created in the previous page.
In Duckietown, Python code must belong to a Python package. 
Python packages are placed inside the directory `packages/` you can find at the root of `my-project`. 
Let us go ahead and create a directory called `my_package` inside `packages/`.

    mkdir -p ./packages/my_package

A Python package is simply a directory containing a special file called `__init__.py`. 
So, let us turn that `my_package` into a Python package.

    touch ./packages/my_package/__init__.py

Now that we have a Python package, we can create a Python script in it. 
Use your favorite text editor or IDE to create the file `./packages/my_package/my_script.py` and 
place the following code inside it.


```python
message = "\nHello World!\n"
print(message)
```

We now need to tell Docker we want this script to be the one executed when we run the command 
`dts devel run`. In order to do so, open the file `./launchers/default.sh` and replace the line

``` 
echo "This is an empty launch script. Update it to launch your application."
```

with the line

``` 
dt-exec python3 -m "my_package.my_script"
```

```{note}
Always prepend `dt-exec` to the main command in `./launchers/default.sh`.

Using `dt-exec` helps us deal with an interesting problem called "The zombie reaping problem" 
(more about this in this [article][article]).

[article]: https://blog.phusion.nl/2015/01/20/docker-and-the-pid-1-zombie-reaping-problem/
```
 
You can also create more launcher scripts. To know more about that, check out the page [](dtproject-launchers).    
 
Let us now re-build the image:

    dts devel build -f 

and run it:

    dts devel run

This will show the following message:

```
...
==> Launching app...

Hello World!

<== App terminated!
```

```{admonition} Congratulations 🎉
You just built and run your own Duckietown-compliant Docker image.
```
