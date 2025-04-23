```{seo}
:description: Learn the basics of using a terminal for robotics and how the Duckietown Shell (`dts`) can enhance your development workflow.
:keywords: terminal, Duckietown, dts shell, robotics, terminal commands, Duckiebot, development tools, BASH
```


(sec:terminal-basics)=
# Basics - Terminal

```{needget}
* [Laptop setup](book-opmanual-duckiebot:laptop-setup)
* [Duckietown account](book-opmanual-duckiebot:dt-account)
---
* Know how to use a terminal
```


Working over the terminal is a skill that every roboticist-to-be needs to acquire. It enables working on other computers remotely without the need for a graphical user interface (GUI), and after some practice, it actually increases working efficiency. Moreover, although not recommended at the very beginning, terminal-based workflows can be highly customized for additional efficiency as well as duckie-cool factor.   


(sec:using-terminal)=
## Using a terminal 

It makes sense to learn how to use the terminal very well, as it will save you a lot of time along the way. 
If you are completely new to working with a terminal, often also called "console" or "command line", an official beginners
tutorial can be found [on the Ubuntu website](https://tutorials.ubuntu.com/tutorial/command-line-for-beginners#0).

The terminal requires using a conding language called [BASH](https://en.wikipedia.org/wiki/Bash_(Unix_shell)), or "Bourne Again SHell". BASH is completely free and works on all major operating systems (e.g., Linux, macOS, Windows). If you are looking for an extensive list of commands that can be used from the terminal, [this command line cheatsheet](https://ss64.com/bash/) is a good resource.

```{todo}
1. write section on how to pimp up the terminal; 2. write section on IDEs and provide a preset configuration for Pycharme CE and VisualStudio; 3. Provide some best practices for development; 4. Arguably move this away from the developer manual. It's too basic. 
```

(sec:using-dt-shell)=
## Using the Duckietown Shell (`dts`)

The Duckietown Shell, or `dts` for short, is a pure Python, easily distributable (few dependencies) utility for Duckietown.

The underlying idea is that most of the Duckietown functionalities are implemented as Docker containers, and `dts` provides a simplified UI, sparing us the hassle of typing very long `docker run` commands. 

These functionalities range from calibrating your Duckiebot's camera or wheels and running demos, to building this documentation (a.k.a., the Duckumentation), submitting agents to challenges and monitoring your evaluations and leaderboard positioning, or developing and deploying your own code in Duckietown agents. 

Before proceeding further, ensure the [laptop setup instructions](book-opmanual-duckiebot:laptop-setup) have been followed and `dts` is installed on your system. If not, now is the time to go back and do it.