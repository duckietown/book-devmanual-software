```{seo}
:description: Understand how to configure and extend launcher scripts for Duckietown DTProject Docker containers.
:keywords: Duckietown, DTProject, launchers, Docker entry scripts, dt-launcher
```

(dtproject-launchers)=
# Launchers

As previously described, the `launchers/` directory contains entry scripts for Docker containers spawned from a DTProject image.

```{attention}
**If you are familiar with Docker.**

We purposefully refer to these scripts as **entry scripts** as not to confuse them with **entrypoints** as they are defined by Docker. 
In fact, launcher scripts **will not** be directly used as container entrypoints. A generic entrypoint is defined in all Duckietown images and is used to configure the containers' network, environment, etc. 
```

(dtproject-launcher-default)=
## The default launcher

Every project template includes a launcher file called `default.sh`, which runs when executing `dts devel run`. Modify this script to launch the main behavior, for example, an autonomous lane-following pipeline.

While having a default launcher is great, often projects can use more than a single entry point.

Additional launchers can exercise individual components or tests. For example, leave `default.sh` unchanged for the full pipeline and create `camera-intrinsic-calibration.sh`, `camera-extrinsic-calibration.sh`, etc., to perform specific camera calibrations.

(dtproject-launcher-add-new)=
## Add new launchers

To add a launcher, create a file in `launchers/` with either a `.sh` extension or a valid [shebang](https://en.wikipedia.org/wiki/Shebang_(Unix)) on the first line.

For example, this Python script file `launchers/my-launcher.py` can be a launcher:

```python
#!/usr/bin/env python3

message = "This is a Python launcher"
print(message)
```

:::{note}
After `dts devel build`, the output lists installed launchers:

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
