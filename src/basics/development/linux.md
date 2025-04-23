```{seo}
:description: Learn about Linux and Ubuntu, including installation methods for dual boot or virtual machine setups, and the basics of working with the terminal.
:keywords: Linux, Ubuntu, installation, dual boot, virtual machine, terminal, Duckietown
```

(sec:developer_basics_linux)=
# Linux distributions

This section introduces Linux distributions, with a focus on the Ubuntu distribution. You will learn how to install Ubuntu in _dual-boot_ mode or inside a _virtual machine_.

## Linux kernel

Linux refers to a family of free and open-source operating systems built around the Linux kernel, first released in 1991. Typically, Linux is packaged in distributions such as Fedora or Ubuntu.

The Duckietown community officially supports _Ubuntu_ as the Linux distribution.

## Ubuntu

There are Ubuntu release every few years. For example, Ubuntu 22.04 LTS (Long-Term Support), released in 2022, will be supported until April 2032.

## Installation

We recommend installing Ubuntu directly on your laptop or as a dual-boot operating system alongside your existing OS. If you prefer, we also provide guidance on installing Ubuntu within a virtual machine.

### Dual Boot Ubuntu Installation

* First you need to download a `.iso` image file which contains the version of Ubuntu you want. 
    Here is [22.04 LTS](http://releases.ubuntu.com/22.04/) make sure to download the desktop image, and one of the [architecture that matches your computer](https://help.ubuntu.com/community/SupportedArchitectures#:~:text=Ubuntu%20is%20currently%20officially%20compatible,x86%2C%20PowerPC%2C%20and%20SPARC64.). 
* Next, you need a free USB drive with at least 2GB of space. The drive will be completely written over.
* You need some software to write the .iso to the USB. If on Windows you can use [Rufus](https://rufus.ie/), if on macOS [Balena Etcher](https://etcher.balena.io/)
* Create the bootable USB drive, disconnect the USB then reconnect to your computer.
* Restart your computer:
  * If your computer boots into the existing operating system, you will need to change the boot order in your BIOS. 
  * Restart your computer again and press the key that accesses the BIOS during startup. This key varies by model (e.g., F1 or F2 for Lenovo laptops). Consult your laptop’s manual or search online for guidance
  * Change the boot order to prioritize the USB drive.
  
* Your computer should now boot into the Ubuntu installation. Follow the instructions for dual boot setup.

### Virtual Machine

* Download the desired Ubuntu `.iso` image file. 
    The [22.04 LTS](http://releases.ubuntu.com/22.04/) desktop image is recommended.
* Download and install your preferred Virtual Machine platform, such as VirtualBox or VMware. [UTM](https://mac.getutm.app/) is a good choice if you have an M1-M4 mac. 

```{note}
When using a Virtual Machine, specific networking settings may need to be adjusted. ***The virtual machine must appear as a device on your local network***. For example, in VirtualBox, you will need to set up a **Bridged Network**, though this configuration might differ for other hypervisors.
```

```{attention}
The Virtual Machine path is possible, but definitely less straightforward than the dual-boot installation, as it adds an extra layer of complexity especially when it comes to networking. We strongly recommend a native/dual-boot Ubuntu installation unless you are comfortable with discomfort. 
```

## Terminal

Some super basic pointers:

* Open a terminal with <keyb>Ctrl</keyb> + <keyb>Alt</keyb> + <keyb>T</keyb>
* The `/` directory is the root directory, containing all other directories.
* The `~` symbol refers to your home directory, located in `/home/[username]`.


## Hands-On

We recommend installing a Linux distribution on your computer and familiarizing yourself with its basic operations before proceeding to the next sections.


## Ask the community

For further questions on software development best practices, consider [joining the Duckietown Slack community](https://duckietown.com/join-slack).
