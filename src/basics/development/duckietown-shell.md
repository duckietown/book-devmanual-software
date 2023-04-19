(sec:developer_basics_dts)=
# Duckietown Shell

This section of the book will introduce the Duckietown Shell 
(`dts` in short) and the reason behind its creation.
In this book, we will use dts commands quite often, make sure you don't 
miss this section.


## Brief History

The Duckietown Shell is indeed a shell. It was created in July 2018 to help
Duckietown users launch Duckietown demos on a Duckiebot. It became clear pretty
soon that having a dedicated shell for Duckietown was a game changer for the
whole community. In fact, since the very beginning, the shell had a built-in
system for auto-update, which allowed developers to develop new commands or
improve old ones and deploy the changes in no time.

Duckietown has a history of using `Makefiles` as a way to simplify complex and
operations involving many (usually very long) bash commands.
Other developers, instead, preferred bash scripts over Makefiles.
And finally, our CI system (based on [Jenkins](https://www.jenkins.io/)), 
used `Jenkinsfiles` to define automated jobs.

The Duckietown Shell came to the rescue and unified everything, while 
Makefiles, bash scripts and Jenkinsfiles slowly started disappearing from our
repositories. Today, Docker images to run on Duckiebots, Python libraries
published on PyPi and even the book you are reading right now are built through 
dts.


(dts_get_started)=
## Get Started

The Duckietown Shell is released as a Python3 package through the PyPi package
store. You can install the Duckietown Shell on your computer by running,

    pip3 install duckietown-shell
    
This will install the `dts` command. 
The Duckietown Shell is distribution independent, so the first time you launch
it you have to specify the distribution of Duckietown software you are working 
on. You can do so by running the command,

    dts --set-version ![DISTRO]
    
where `![DISTRO]` can be any of the official distributions of Duckietown
software, e.g., `daffy`, `ente`.
This will download the commands for the given distribution before the command
prompt is shown.

Use the command,

    (Cmd) commands
    
to list all the commands available to the chosen distribution.

You don't really need to run the shell before you can type in your 
command, for example, you can achieve the same result as above by running,
 
    dts commands

:::{note}
The nice thing about opening the shell before typing your command is that
then you can use the <kbd>Tab</kbd> key to auto-complete.
:::

## Installable commands

Some commands come **not** pre-installed. These are usually commands that are
either very specific to an application, thus not useful to the majority of
Duckietown users, or commands that can only be used during a short time
window, like commands that let you participate to competitions periodically
organized at international AI and Robotics conferences, e.g. 
[AIDO](https://aido.duckietown.org/).


## Hands on

Install the Duckietown Shell as instructed in [](#dts_get_started).
Make sure everything works as expected by running the command
`dts update` successfully.


## Ask the community

If you have any questions about the Duckietown Shell, join and ask on the Duckietown Slack!
