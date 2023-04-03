(sec:developer_basics_linux)=
# Linux

This section of the book will introduce Linux distributions and specifically
the Ubuntu distribution. We will provide guides for installing Ubuntu in 
_dual-boot_ mode or inside a _virtual machine_.

## Linux

Linux is a group of free and open-source software operating systems built 
around the Linux kernel first released in 1991. Typically, Linux is packaged 
in a form known as a Linux distribution such as Fedora or Ubuntu.

Ubuntu is the Linux distribution officially supported by the Duckietown community.


## Ubuntu

As of this writing, the most recent version of Ubuntu is 22.04 LTS (Long Term Service) 
which will be supported until April 2032. Ubuntu 22.04+ versions should also be fine.


## Installation

It is highly recommended to install Ubuntu directly on your laptop or as a dual boot 
operating system alongside your existing OS. However we also provide some guidance on 
installing Ubuntu within a Virtual Environment on your laptop.


### Dual Boot

* First you need to download a `.iso` image file which contains the version of Ubuntu you want. 
    Here is [22.04 LTS](http://releases.ubuntu.com/22.04/) make sure to download the desktop image.
* Next, you need a free USB drive with at least 2GB of space. The drive will be completely written over.
* You need some software to write the .iso to the USB. If on Windows you can use [Rufus](https://rufus.ie/)
* Create the bootable USB drive, disconnect the USB then reconnect to your computer.
* Restart your computer
    - If your computer simply boots into the existing operating system you need to change the boot order in your BIOS.
    - Restart your computer again and press the button during startup which lets you into the BIOS. It may say on your computer what this button is but you may need to Google depending on your laptop model. For example Lenovo might be F1 or F2.
    - Look for an option to change boot order and put priority on your USB drive.
* Your computer should now boot into Ubuntu installation and you can follow the instructions for dual boot.

### Virtual Machine

* First you need to download a .iso image file which contains the version of Ubuntu you want. 
    Here is [22.04 LTS](http://releases.ubuntu.com/22.04/) make sure to download the desktop image.
* Download your desired Virtual Machine platform (popular choices are Virtual Box and VMWare).

Note: Using a Virtual Machine might require some particular settings for you networking settings. The virtual machine should appear as a device on your local network. For example, in VirtualBox, you need to set up a _Bridged Network_. This might differ in other hypervisors.

## Terminal

Some pointers:

* Open a terminal with Ctrl + Alt + T
* `/` is the top level root directoy which contains your
* `~` refers to your home folder located in `/home/[username]`


## Hands on

We suggest that you install a Linux distribution on your computer and get familiar
with it before proceeding to the next sections.


## Ask the community

If you have any questions about good practices in installing Ubuntu on your computer or
other questions about Ubuntu, join and ask on the Duckietown Slack!
