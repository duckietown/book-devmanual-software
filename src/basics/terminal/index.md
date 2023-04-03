(sec:terminal-basics)=
# Basics - Terminal

```{needget}
* [Laptop setup](book-opmanual-duckiebot:laptop-setup)
* [Duckietown account](book-opmanual-duckiebot:dt-account)
---
* Know how to use a terminal
```


Working over the terminal is a skill that every roboticist-to-be needs to acquire. It enables you to work on remote
 agents or computers without the need for a graphical user interface (GUI) and lets you work very efficiently. Once you get the hang of it, you will find out for yourself how it can make your life easier.  


(sec:using-terminal)=
## Using a terminal 

 It makes sense to learn how to use the terminal very well, as it will save you a lot of time along the way. 
 If you are completely new to working with a terminal, often also called "console" or "command line", an official beginners
 tutorial can be found [on the Ubuntu website](https://tutorials.ubuntu.com/tutorial/command-line-for-beginners#0). 

<!-- 
A list of commands that are frequently used can be found in the [appendix](#useful-linux-commands).
-->

If you are looking for an extensive list of commands that can be used from the terminal, [this](https://ss64.com/bash/) is the place to look at.

```{todo}
write section on how to pimp up the terminal and provide some best practices for development
```

(sec:using-dt-shell)=
## Using the Duckietown Shell

The Duckietown Shell, or `dts` for short, is a pure Python, easily distributable (few dependencies) utility for Duckietown.

The idea is that most of the functionalities are implemented as Docker containers, and `dts` provides a nice interface for
that, so that users should not type a very long docker run command line. These functionalities range from calibrating
your Duckiebot and running demos to building this Duckumentation and submitting agents to open challenges and monitoring your evaluations and leaderboard positioning. 

If you followed all the steps in the [laptop setup](book-opmanual-duckiebot:laptop-setup), you already installed  `dts`. If not, now is the time to go back and do it.
