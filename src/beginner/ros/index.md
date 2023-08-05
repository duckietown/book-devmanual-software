(dtproject-ros)=
# Beginner - Use ROS

```{needget}
* A computer set up with the [Duckietown software requirements](book-opmanual-duckiebot:laptop-setup)
* An initialized [Duckiebot](book-opmanual-duckiebot:get-db-hw)
* [Completed tutorial on DTProject](dtproject)
---
* A working knowledge of ROS development in Duckietown
* A custom ROS node with a publisher and a subscriber running on your Duckiebot
```

(basic-structure)=
## Basic Project Structure
In Duckietown, everything runs in Docker containers. All you need in order to run a piece of 
software that uses ROS in Duckietown is a Duckietown-compliant Docker image with your software in it.

A boilerplate is provided [here](https://github.com/duckietown/template-ros). 
If you have completed the [Tutorial on DTProject](dtproject) (as you should have), you will recognize
the content of this repository as being a DTProject template.

Similarly to what we learned in [](dtproject-create-new), we will instantiate a new project repository by 
using the template repository available on GitHub.

The procedure for building and executing a ROS-compatible DTProject is the same we learned in the
[Tutorial on DTProject](dtproject) section.

Let's get started!
