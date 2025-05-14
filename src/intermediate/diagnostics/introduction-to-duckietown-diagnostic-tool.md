```{seo}
:description: Understand when and why to use the Duckietown Diagnostics tool, distinguishing steady-state and transient-state measurements to evaluate resource usage on a Duckiebot.
:keywords: Duckietown diagnostics, performance profiling, steady state, transient state, Raspberry Pi monitoring, resource footprint
```

(sec:sw_diagnostics_intro)=
# Introduction to the Diagnostics Tool

```{needget}
* Ability to launch and modify ROS nodes on a Duckiebot  
* Basic familiarity with Raspberry Pi or Jetson Nano resource constraints
---
* Decide whether to run a steady-state or transient-state diagnostic session  
* Interpret the resulting resource-usage traces
```

The *Duckietown Diagnostics* utility periodically records CPU, memory, temperature, I/O, and network metrics while an experiment is running. By replaying these snapshots, you can verify that new code respects the onboard computer tight resource budget and does not introduce hidden side effects.

(devel_sw_diagnostics_example)=
## Running example

Assume a **single-camera Duckiebot** whose driver publishes frames at **20 Hz**. We plan to raise the rate to **30 Hz** and must quantify how this change impacts resource usage.

---

## Why should diagnostics be run?

Execute the tool **each time you modify code** and need to gauge its footprint on system resources.

*Expected consequences*  
- Higher frame rate → more CPU time, RAM, and bus bandwidth  
- Possible *secondary* effects: increased SoC temperature, extra network traffic if frames are forwarded downstream

Diagnostics offers a repeatable method for measuring and analyzing these effects.

---

## Steady-state and transient-state sessions

The tool supports two common scenarios:

| Scenario | Purpose | When to launch / how long to run |
|----------|---------|----------------------------------|
| **Steady-state analysis** | Detect slow drifts such as memory leaks | Start *after* the system settles; record for an extended interval |
| **Transient-state analysis** | Observe short bursts triggered by an event | Start anytime; record for *t* > *T* to include at least one event cycle |

### Example – tuning camera FPS
* **Steady state**: run diagnostics overnight and inspect RAM usage to reveal leaks at 30 Hz.  
* **Transient state**: capture several seconds around a frame-grab burst to confirm CPU spikes remain acceptable.

By selecting the appropriate window, actionable insight can be obtained without overwhelming the storage with unnecessary data.



<!--
(sec:sw_diagnostics_intro)=
# Introduction


(devel_sw_diagnostics_example)=
## Running example

Throughout this section we will refer to the toy example of a robot
with a single camera (just like our Duckiebots) in which camera drivers 
produce image frames at a frequency of `20Hz` and we are interested in
pushing the camera to its limit, i.e., `30Hz`.


## When do I need it?

You need to run the diagnostics tool every time you have made changes
to a piece of code and you want to test how these changes affect the 
footprint of your code on the system resources and the system as a whole.

Considering our toy example above, we expect that changing the drivers
frequency will likely result in higher usage of resources in order to
cope with the increase in images that need to be processed.
Sometimes these changes have a direct and expected effect on the 
system's resources, e.g., CPU cycles, RAM, etc. Others, they have effects
that are legitimate from a theoretical point of view but hard to
exhaustively enumerate a priori, e.g., increase in CPU temperature due
to higher clock frequencies, increase in network traffic if the images 
are transferred over the network.

The diagnostics tool provides a standard way of analyzing the response of
a system to a change.


## When do I run it?

The diagnostics tool is commonly used for two use cases:

- analysis of _steady states_ (long-term effects) of a system;
- analysis of _transient states_ (short-term effects) of a system;

The _steady state_ analysis consists of measuring the activity of a system
in the long run and in the absence of anomaly or changes. For example,
if we want to check for memory leaks in a system, we would run a _steady state_
analysis and look at the RAM usage in a long period of time.
In this case, we would run the diagnostics tool only after the system reached a 
stable (steady) state and we don't expect significant events to happen.

The _transient state_ analysis consists of measuring how a system reacts to
a change in the short run. For example, you have a process that receives 
point clouds from a sensor and stores them in memory to perform ICP alignment 
on them every `T` seconds. In this case, we expect that this process will be 
fairly inactive in terms of CPU usage for most of the time with periodical spikes
every `T` seconds. Clearly, small values of `T` mean fewer point clouds to align 
every time ICP fires but more frequent alignments while large values of `T` mean
longer queues of point clouds to align every time ICP fires. 
We might be interested in tuning the value of `T` so that those
spikes do not starve other processes of resources while still maximizing `T`.
In this case, we would monitor the system around those ICP events for different
values of `T`.
In this case, we would run the diagnostics tool at any point in time for a 
duration of `t > T` seconds so that at least one event of interest (e.g., ICP event) 
is captured.
-->