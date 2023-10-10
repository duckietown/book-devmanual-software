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

(dtproject-launcher-default)=
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


(dtproject-launcher-add-new)=
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


## A typical example launcher script explained

In this section, we take a look at a simple and typical launcher script. In particular, let's try to understand these utilities:

* `dt-launchfile-init`
* `dt-exec`
* `dt-launchfile-join`

Here is an example launcher script in [dt-commons](https://github.com/duckietown/dt-commons/blob/cb09dcb933e5b304ff73425c867fcc220fd61530/launchers/default.sh):

```
#!/bin/bash
source /environment.sh

dt-launchfile-init
dt-exec echo "This is an empty launch script. Update it to launch your application."
dt-launchfile-join
```

Let's take a look at the file line by line, after the *shebang* (`#!/bin/bash`) line .


```
source /environment.sh
```

The above line loads our custom wrappers/functions facilitating launching programs. They are defined in [this file](https://github.com/duckietown/dt-commons/blob/cb09dcb933e5b304ff73425c867fcc220fd61530/assets/environment.sh), among which the reader in most cases only need to care about the use of the following 3 utilities.

```
dt-launchfile-init
```

The above makes sure the terminating signals are registered correctly for the programs that are run in this script. And it prints the line that goes `"==> Launching app..."`.

Here is what's meant by "register terminating signals": It is common to press `[Ctrl+C]` to stop a terminal program. What happens under the hood is a [signal](https://manpages.ubuntu.com/manpages/focal/en/man7/signal.7.html) is sent to the foreground process that runs in the terminal. The process then decides what to do upon receiving one. And with `dt-launchfile-init`, it is setup such that the `SIGINT` signal will be passed to all child processes of the launcher script. In particular, when `[Ctrl+C]` is captured from the terminal, or when one performs `docker stop ...` to kill a container that runs a launcher script, this utility helps "broadcast" `SIGINT` for its child processes to shutdown properly.

It is common to have some clean-up handling when `SIGINT` is received, e.g. `on_shutdown` function of classes inheriting from [DTROS](sec:intermediate-dtros). That is why it is important to ensure this signal gets through to all the child processes.

```
dt-exec ...
```

The `dt-exec` function allows us to make new child processes and have them run in the background. You can have as many child processes as you want (i.e., have multiple commands of the form `dt-exec ...`). They all be kept running in the background and warned by `dt-launchfile-init` when a signal is received. Use `dt-exec` only for long running processes, i.e., processes that run until the container is stopped.

```
dt-launchfile-join
```

The above utility waits for any child processes to finish and return. The child processes should be shutdown properly with the help from `dt-launchfile-init`. And once they all exited, the following will be printed in the console: `"<== App terminated!"`.

In summary, these 3 utilities help better manage the signals and processes within the launcher script, and should be used for your launcher scripts as well.
