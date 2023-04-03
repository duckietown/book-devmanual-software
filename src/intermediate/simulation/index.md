(duckietown-simulation)=
# Simulation in Duckietown

```{needget}
* [Implementing Basic Robot Behaviors](#part:basic-robot-behavior)
--- 
* Results: Experience with running and testing on the Duckietown simulator 
```

```{figure} ../../_images/simplesim_free.png
:name: fig:simplesim_free

```

## Why simulation?
Daphne is an avid Duckietowner who loves Duckies. In her mission to "save the Duckies" from bugs in her code she used to spend a large portion of her time writing unit tests for her algorithms and ROS nodes. Some of these tests would check that the accuracy of her object detection pipeline was above a certain threshold, that the estimated offset of the Duckiebot from the lane given several input images was correct or that the output of the controller given several offsets gave sensible results. She noticed that this way of testing would fall short in several aspects:

- The number of hand-crafted edge cases was not representative of the number of situations the Duckiebot would encounter in a single drive
- Issues at the interface of these algorithms would not be caught
- To increase code coverage and maintain it, a lot of time would need to go into writing tests, mock ups, gathering and labelling test data, etc
- Quantifying controller performance was hard without having access to a model of the vehicle used to propagate the state forward in time

Daphne also found that having to charge her robot's battery, setting up her Duckietown loop, placing her Duckiebot on the loop, connecting to it, and running the part of the pipeline that had to be tested everytime she or someone in her team wanted to merge new changes into the codebase was extremely time consuming.

More over, Daphne and her real Duckiebot only have access to a small Duckietown loop. But she wants to ensure that her algorithms work in the most complicated and busy environments of Duckietown.

All of the above were compelling reasons for Daphne to start looking at full-stack simulators that would allow her to simultaneously address the shortcomings of unit testing, the inconvenience of manual testing and the ability to test scenarios that are not possible or too risky in real life. 

Luckily, she found just the right thing at the [Duckietown gym](https://github.com/duckietown/gym-duckietown).

Daphne's story is the story of every autonomous driving company, whose mission is instead to "save the humans" and which cannot afford to make mistakes on the real roads, and which require automated integration testing tools that can be run faster-than-real-time under challenging conditions. As an example, Waymo has driven around 20 million miles on real roads, but around 15 billion miles in simulation!

## Introduction to the Duckietown Simulator

Gym-Duckietown is a simulator for the [Duckietown](https://duckietown.org) universe, 
written in pure Python/OpenGL (Pyglet). It places your agent, a Duckiebot, 
inside an instance of a Duckietown: a loop of roads with turns, 
intersections, obstacles, Duckie pedestrians, and other Duckiebots.

Gym-Duckietown is fast, open, and incredibly customizable. What started as a lane-following simulator has evolved into a fully-functioning autonomous driving simulator that you can use to train and test your Machine Learning, Reinforcement Learning, Imitation Learning, or even classical robotics algorithms. Gym-Duckietown offers a wide range of tasks, from simple lane-following to full city navigation with dynamic obstacles. Gym-Duckietown also ships with features, wrappers, and tools that 
can help you bring your algorithms to the real robot, including [domain-randomization](https://blog.openai.com/spam-detection-in-the-physical-world/), accurate camera distortion, and differential-drive physics (and most importantly, realistic waddling).

```{figure} ../../_images/finalmain.gif
:name: fig:finalmain-sim

```

(simulator_quickstart)=
## Quickstart Guide

To run a minimal demo of the simulator, you simply need a (virtual) environment 
with the gym_duckietown pip3 package installed.

To set up such an environment, the safest way is to run the following 
(you could also skip the virtual environment but you may have clashing packages 
installed):

    $ cd ~ && virtualenv dt-sim
    $ source dt-sim/bin/activate
    $ pip3 install duckietown-gym-daffy

Now you need to create a simple python script with uses the gym-duckietown API 
to connect to the simulator, the API is very simple as you will see.

Create and run the following file, from within the environment you have set up above:

```python
#!/usr/bin/env python3
import gym_duckietown
from gym_duckietown.simulator import Simulator
env = Simulator(
        seed=123, # random seed
        map_name="loop_empty",
        max_steps=500001, # we don't want the gym to reset itself
        domain_rand=0,
        camera_width=640,
        camera_height=480,
        accept_start_angle_deg=4, # start close to straight
        full_transparency=True,
        distortion=True,
    )   
while True:
    action = [0.1,0.1]
    observation, reward, done, misc = env.step(action)
    env.render()
    if done:
        env.reset()
```

What do you observe? Does this make sense? Why is it driving straight? 
Can you make it drive backwards or turn? When is `done = True`? What is `observation`? 


### Driving around in the simulator

If you want to drive the robot around in simulation you might have read about the utility script `manual_control.py`. This is located in the root of the [gym_duckietown](https://github.com/duckietown/gym-duckietown) repository and can be run after making sure that all the dependencies are met. Clone the repository and in the root of it run:

    $ ./manual_control.py --env-name Duckietown-udem1-v0

You should be able to drive around with the arrow keys. 
If you are experiencing large delays and low frame rate, please replace the lines

```python

pyglet.clock.schedule_interval(update, 1.0 / 30)

# Enter main event loop
pyglet.app.run()

```

by

```python
import time

...

dt = 0.01
while True:
    update(dt)
    time.sleep(dt)

```

## Environments
There are multiple registered gym environments, each corresponding to a 
different [map file](https://github.com/duckietown/gym-duckietown/tree/master/gym_duckietown/maps):

* `Duckietown-straight_road-v0`
* `Duckietown-4way-v0`
* `Duckietown-udem1-v0`
* `Duckietown-small_loop-v0`
* `Duckietown-small_loop_cw-v0`
* `Duckietown-zigzag_dists-v0`
* `Duckietown-loop_obstacles-v0` (static obstacles in the road)
* `Duckietown-loop_pedestrians-v0` (moving obstacles in the road)

The `MultiMap-v0` environment is essentially a [wrapper](https://github.com/duckietown/gym-duckietown/blob/master/gym_duckietown/envs/multimap_env.py) for the simulator which
will automatically cycle through all available [map files](https://github.com/duckietown/gym-duckietown/tree/master/gym_duckietown/maps). This makes it possible to train on
a variety of different maps at the same time, with the idea that training on a variety of
different scenarios will make for a more robust policy/model.

`gym-duckietown` is an _accompanying_ simulator to real Duckiebots, 
which allow you to run your code on the real robot. 
We provide a domain randomization API, which can help you transfer your 
trained policies from simulation to real world. Without using a domain transfer 
method, your learned models will likely overfit to various aspects of the simulator, 
which won't transfer to the real world. When you deploy, you and your 
Duckiebot will be running around in circles trying to figure out what's going on.


```{figure} ../../_images/domainrand-sim.gif
:name: fig:domainrand-sim

```


## Installation

We have covered the basic installation in the [quickstart guide](simulator_quickstart).

### Alternative Installation Instructions (Alternative Method)

Alternatively, you can find further installation instructions [here](https://github.com/duckietown/gym-duckietown.git)

### Docker Image

There is a pre-built Docker image available [on Docker Hub](https://hub.docker.com/r/duckietown/gym-duckietown), which also contains an installation of PyTorch.

```{note}
In order to get GPU acceleration, you should install and use [nvidia-docker](https://github.com/NVIDIA/nvidia-docker).
```


## Design

### Map File Format

The simulator supports a YAML-based file format which is designed to be easy to hand edit. See the [maps subdirectory](https://github.com/duckietown/gym-duckietown/blob/master/gym_duckietown/maps) for examples. Each map file has two main sections: a two-dimensional array of tiles, and a listing of objects to be placed around the map. The tiles are based on the [Duckietown appearance specification](https://docs.duckietown.org/daffy/opmanual_duckietown/out/duckietown_specs.html).

The available tile types are:

* empty
* straight
* curve_left
* curve_right
* 3way_left (3-way intersection)
* 3way_right
* 4way (4-way intersection)
* asphalt
* grass
* floor (office floor)

The available object types are:

* barrier
* cone (traffic cone)
* duckie
* duckiebot (model of a Duckietown robot)
* tree
* house
* truck (delivery-style truck)
* bus
* building (multi-floor building)
* sign_stop, sign_T_intersect, sign_yield, etc. (see [meshes subdirectory](https://github.com/duckietown/gym-duckietown/blob/master/gym_duckietown/meshes) )

Although the environment is rendered in 3D, the map is essentially two-dimensional. As such, objects coordinates are specified along two axes. The coordinates are rescaled based on the tile size, such that coordinates [0.5, 1.5] would mean middle of the first column of tiles, middle of the second row. Objects can have an `optional` flag set, which means that they randomly may or may not appear during training, as a form of domain randomization.

### Observations

The observations are single camera images, as numpy arrays of size (120, 160, 3). These arrays contain unsigned 8-bit integer values in the [0, 255] range.
This image size was chosen because it is exactly one quarter of the 640x480 image resolution provided by the camera, which makes it fast and easy to scale down
the images. The choice of 8-bit integer values over floating-point values was made because the resulting images are smaller if stored on disk and faster to send over a networked connection.

### Actions

The simulator uses continuous actions by default. Actions passed to the `step()` 
function should be numpy arrays containining two numbers between -1 and 1. 
These two numbers correspond to the left and right wheel input respectively. 
A positive value makes the wheel go forward, a negative value makes it go backwards. 
There is also a [Gym wrapper class](https://github.com/duckietown/gym-duckietown/blob/daffy/gym_duckietown/wrappers.py) named `DiscreteWrapper` which allows you to use discrete actions (turn left, move forward, turn right) instead of continuous actions if you prefer.

### Reward Function

The default reward function tries to encourage the agent to drive forward along 
the right lane in each tile. Each tile has an associated Bezier curve defining 
the path the agent is expected to follow. The agent is rewarded for being as 
close to the curve as possible, and also for facing the same direction as the 
curve's tangent. The episode is terminated if the agent gets too far outside of 
a drivable tile, or if the `max_steps` parameter is exceeded. See the `step` 
function in [this source file](https://github.com/duckietown/gym-duckietown/blob/daffy/gym_duckietown/envs/simplesim_env.py).

## Customizing the Simulator

You can modify the parameters of the simulator. 
Simply modify the parameters sent to the `Simulator` constructor:

```
        from gym_duckietown.simulator import Simulator
        env = Simulator(
            seed=123, # random seed
            map_name="loop_empty",
            max_steps=500001, # we don't want the gym to reset itself
            domain_rand=0,
            camera_width=640,
            camera_height=480,
            accept_start_angle_deg=4, # start close to straight
            full_transparency=True,
            distortion=True,
        )
```

When we [take a look at the constructor](https://github.com/duckietown/gym-duckietown/blob/aido2_lf_r1/gym_duckietown/simulator.py#L145-L180), you'll notice that we aren't using all of the parameters listed. In particular, the three you should focus on are:
    
- `map_name`: What map to use;
- `domain_rand`: Applies domain randomization, a popular, black-box, sim2real technique
- `randomized_maps_on_reset`: Slows training time, but increases training variety.
- `camera_rand`: Randomizes the camera calibration to increase variety.
- `dynamics_rand`: Simulates a miscalibrated Duckiebot, to better represent reality.

(simulator-running-headless)=
### Running headless

The simulator uses the OpenGL API to produce graphics. This requires an X11 display to be running, which can be problematic if you are trying to run training code through on SSH, or on a cluster. You can create a virtual display using [Xvfb](https://en.wikipedia.org/wiki/Xvfb). The instructions shown below illustrate this. Note, however, that these instructions are specific to MILA, look further down for instructions on an Ubuntu box:

```zsh
# Reserve a Debian 9 machine with 12GB ram, 2 cores and a GPU on the cluster
sinter --reservation=res_stretch --mem=12000 -c2 --gres=gpu

# Activate the gym-duckietown Conda environment
source activate gym-duckietown

cd gym-duckietown

# Add the gym_duckietown package to your Python path
export PYTHONPATH="&#36;{PYTHONPATH}:`pwd`"

# Load the GLX library
# This has to be done before starting Xvfb
export LD_LIBRARY_PATH=/Tmp/glx:&#36;LD_LIBRARY_PATH

# Create a virtual display with OpenGL support
Xvfb :&#36;SLURM_JOB_ID -screen 0 1024x768x24 -ac +extension GLX +render -noreset &#38;<code>&gt;</code> xvfb.log &#36;
export DISPLAY=:&#36;SLURM_JOB_ID

```


## Troubleshooting

If you run into problems of any kind, don't hesitate to [open an issue](https://github.com/duckietown/gym-duckietown/issues) on this repository. It's quite possible that you've run into some bug we aren't aware of. Please make sure to give some details about your system configuration (ie: PC or Max, operating system), and to paste the command you used to run the simulator, as well as the complete error message that was produced, if any.


```{trouble}
ImportError: Library "GLU" not found
---
You may need to manually install packaged needed by Pyglet or OpenAI Gym on your system. The command you need to use will vary depending which OS you are running. For example, to install the glut package on Ubuntu:

``
    $ sudo apt-get install freeglut3-dev
``

And on Fedora:

``
    $ sudo dnf install freeglut-devel
``
```

```{trouble}
NoSuchDisplayException: Cannot connect to "None"
---
If you are connected through SSH, or running the simulator in a Docker image, 
you will need to use xvfb to create a virtual display in order to run the simulator. 
See the [Running Headless](simulator-running-headless) subsection.
```

```{trouble}
Poor performance, low frame rate
---
It's possible to improve the performance of the simulator by disabling Pyglet error-checking code. Export this environment variable before running the simulator:

``
    $ export PYGLET_DEBUG_GL=True
``
```

```{trouble}
Unknown encoder 'libx264' when using gym.wrappers.Monitor
---
It is possible to use `gym.wrappers.Monitor` to record videos of the agent performing a task. See [examples here](https://www.programcreek.com/python/example/100947/gym.wrappers.Monitor).

The libx264 error is due to a problem with the way ffmpeg is installed on some linux distributions. One possible way to circumvent this is to reinstall ffmpeg using conda:

``
      $ conda install -c conda-forge ffmpeg
``

Alternatively, screencasting programs such as [Kazam](https://launchpad.net/kazam) can be used to record the graphical output of a single window.

```

## How to cite
Please use this bibtex if you want to cite this repository in your publications:

```
@misc{gym_duckietown,
  author = {Chevalier-Boisvert, Maxime and Golemo, Florian and Cao, Yanjun and Mehta, Bhairav and Censi, Andrea and Paull, Liam},
  title = {Duckietown Environments for OpenAI Gym},
  year = {2018},
  publisher = {GitHub},
  journal = {GitHub repository},
  howpublished = {\url{https://github.com/duckietown/gym-duckietown}},
}
```
