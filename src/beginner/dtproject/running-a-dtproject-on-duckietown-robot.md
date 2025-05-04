```{seo}
:description: Learn how to build and deploy a Duckietown DTProject Docker image directly on a Duckiebot using dts commands.
:keywords: Duckietown, Duckiebot, DTProject, Docker, dts devel, robot deployment, robotics development
```

(dtproject-run-on-duckiebot)=
# How to run a DTproject on your Duckiebot

This section demonstrates how to build and execute a Duckietown DTProject image on the [Duckiebot](https://get.duckietown.com/products/duckiebot-db21), instead of the local machine.

The following assumes a Duckiebot with host name  `ROBOT_NAME` is up and running. First, verify network connectivity to the robot (replace `ROBOT_NAME` with the actual hostname):

    ping ROBOT_NAME.local

If the robot responds, proceed to modify `my_script.py` created previously in [](dtproject-add-your-code) to:

```python
import os

vehicle_name = os.environ['VEHICLE_NAME']
message = f"\nHello from {vehicle_name}!\n"
print(message)
```

Next, build the Docker image directly on the Duckiebot with:

    dts devel build -f -H ROBOT_NAME

The additional parameter `-H ROBOT_NAME` informs `dts` on which host to build the image. After the image is built, run it on the robot with:

    dts devel run -H ROBOT_NAME

Expected successful output:

```
...
==> Launching app...

Hello from ROBOT_NAME!

<== App terminated!
```

:::{note}
Some WARNING and INFO messages from `dts` and from the container entrypoint are normal, for example:

From `dts`
* `WARNING:dts:Forced!`
* `INFO:dts:Running an image for arm64v8 on aarch64.`

Among entrypoint logs of the container:
* `INFO: The environment variable VEHICLE_NAME is not set. Using 'myduckiebot'.`
:::


```{admonition} Congratulations 🎉
You just successfully built and run a Duckietown-compliant and Duckiebot-compatible Docker image. Let this be the first of many!
```
