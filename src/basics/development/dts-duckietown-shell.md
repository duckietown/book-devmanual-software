```{seo}
:description: Introduction to the Duckietown Shell (dts), its history, installation steps, and key usage patterns for robotics development.
:keywords: Duckietown Shell, dts, command‑line, robotics development, installation guide, Python package, Duckietown tools, Duckietown trivia, Duckietown history
```

(sec:developer_basics_dts)=
# Duckietown Shell

The Duckietown Shell (`dts`, in short) exists to make doing robotics more joyful. In other words, it abstracts some of the nitty-gritty details like (very) long Docker commands, into shorter, clearer instructions. 


## Very Brief History of the Duckietown Shell (`dts`)

The Duckietown Shell is, as the name suggests, a shell, i.e., a wrapper around other code. 

It was originally created, in July 2018, to help Duckietown users launch [Duckietown demos](book-opmanual-duckiebot:db-supported-demo-index) on Duckiebots with a single line of code. 

It soon became clear that having a dedicated shell was a game changer for the whole community. For example, the built-in auto-update system allowed contributors to develop new commands, or improve old ones, and deploy the changes in no time. Moreover, it unified different styles of contributions:

* The original distribution of Duckietown software used `Makefiles` as a way to simplify operations involving many (usually very long) bash commands.

* Some early contributors preferred bash scripts over Makefiles.

* Our CI system (based on [Jenkins](https://www.jenkins.io/)), 
used `Jenkinsfiles` to define automated jobs.

Today, `dts` manages the Docker images that run on Duckiebots, Python libraries published on PyPi, and even this very book you are reading right now.


(dts_get_started)=
## Get Started

The Duckietown Shell is released as a Python3 package through the PyPi package
store. Install it with:

    pip3 install duckietown-shell
     
The command installs the executable `dts`.
Because the shell is distribution‑independent, the first launch prompts to specify which Duckietown software distribution (e.g., `daffy`, `ente`) is used.

List the available commands available in the current distribution with:

    (Cmd) commands
    
Launching the interactive shell is optional; you may prepend `dts` directly:
 
    dts commands

:::{tip}
Opening the shell before typing a command enables autocomplete with <kbd>Tab</kbd>.
:::

## Installable commands

Some commands are not pre‑installed. Typically these commands either target specialised applications that most users do not require or they are tied to time‑sensitive events, such as competitions organised during robotics conferences (for example, AIDO).

Some commands come **not** pre-installed. These are usually commands that are
either very specific to an application, thus not useful to the majority of
Duckietown users, or commands that can only be used during a short time
window, like those used for the past AI Driving Olympics ([AI-DO](https://duckietown.com/research/ai-driving-olympics/).


## Hands-on

Install the Duckietown Shell as instructed in [](#dts_get_started).

Make sure everything works as expected by running the command
`dts update` successfully.

```{tip}
It is a good habit to keep your commands up to date by running `dts update`. 
```


## Ask the community

For questions about `dts` 
[join the Duckietown Slack community](https://duckietown.com/join-slack) and ask away.