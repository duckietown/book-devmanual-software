(part:sw_diagnostics)=
# Intermediate – Diagnostics

```{needget}
* Basic experience running code on the Duckiebot
* Understanding of the [DTROS node structure](sec:intermediate-dtros)
---
* Ability to record and inspect system‑level statistics with the Duckietown Diagnostics tool
```
Duckietown is designed to execute **complex—often state‑of‑the‑art—algorithms on modest hardware**, such as a Raspberry Pi or an NVIDIA Jetson Nano.  
Because computational resources are limited, developers must monitor usage carefully and optimize where possible.
The **Duckietown Diagnostics** utility provides a lightweight way to **record system status over the course of an experiment**.  
Think of it as an automated observer that takes periodic snapshots of CPU load, memory footprint, I/O, and other metrics while your pipeline is running.
Further sections explain how to start a recording session, visualize the data, and integrate the tool into custom experiments.

<!--
(part:sw_diagnostics)=
# Intermediate - Diagnostics

One of the strengths of Duckietown is that of allowing complex 
(sometimes state-of-the-art) algorithms to run on low-end computing
devices like the Raspberry Pi.
Unfortunately, low-end devices are not famous for their computational
power, so we developers have to be smart about the way we use the 
resources available.

The Duckietown Diagnostics tool provides a simple way of _recording_ 
the status of a system during an _experiment_.
The easiest way to think about it is that of an observer taking snapshots 
of the status of our system at regular temporal intervals.

-->


```{tableofcontents}
```