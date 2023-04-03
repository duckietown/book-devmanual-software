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
