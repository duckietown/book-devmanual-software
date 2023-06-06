(dtproject-launchers)=
# Launchers

As we discussed earlier, the directory `launchers/` contains scripts that can serve as entry scripts
to the Docker containers that are spawned out of a DTProject Docker image.

```{attention}
**If you are familiar with Docker.**

We purposefully refer to these scripts as **entry scripts** as not to confuse them with 
**entrypoints** as they are defined by Docker. 
In fact, launcher scripts **will not** be directly used as container entrypoints. A generic entrypoint
is defined in all Duckietown images and is used to configure the containers' network, environment, 
etc. 
```

## The default launcher

Every project template comes with a launcher file called `default.sh`. This is the launcher that runs
by default inside the container when we run the command `dts devel run`.
This launcher should be modified and set to launch the main behavior of our project. For example,
when making a DTProject that implements autonomous lane following in Duckietown, we want our default 
launcher to run the lane following pipeline.

While the default launcher is great, you will quickly realize that, for some projects, having only one 
entry behavior is quite limiting.
For example, let us imagine that we are working on an autonomous lane
following behavior DTProject. In this case, apart from running the full behavior (we will use the default
launcher for that), it might be useful to have entry scripts that allow us to run single components of our
multi-component lane follower pipeline, say, so that we can test single components one at a time. 

In some cases, we will use additional launchers as test scripts. In other cases, it might make sense to
leave the default launcher untouched and only implement additional launchers. This might be useful,
for example, when a DTProject implements a collection of sensor calibrations. In this case, we might
want to leave the default launcher untouched as there is no "default calibration", and then have the 
additional launchers `camera-intrinsic-calibration.sh`,  `camera-extrinsic-calibration.sh`, etc.


## Add new launcher

In order to add a new launcher, you can simply make a new file inside the `launchers/` directory.
The only rule is that such file must either have extension `.sh`, or, have a 
[shebang](https://en.wikipedia.org/wiki/Shebang_(Unix)) 
declaration on its first line.

For example, we can make a launcher out of a Python script file `launchers/my-launcher.py` with the content,

```python
#!/usr/bin/env python3

message = "This is a Python launcher"
print(message)
```

:::{note}
The output from the command `dts devel build` shows the list of launchers installed in the image we 
just built. For example,

```text
...
-------------------------
Image launchers:
 - default
 - test-agent
 - test-lens-distortion
-------------------------
```

:::


## Launchers inside the container

Launchers inside the container appear as shell commands named as `dt-launcher-<LAUNCHER_NAME>`.
For example, the launcher file `launchers/my-launcher.sh` will be available as the shell command
`dt-launcher-my-launcher`.
