(part:devel_modules)=

# Beginner - Software Modules

This section of the book focuses on the concept of software module in Duckietown.

Devices in Duckietown (e.g., Duckiebots, Watchtowers, etc.) are not configured to accept
code directly. The Operating System running on the on-board computers is configured
to accept only code running inside Docker containers. Modules are an easy, robust and effective way of wrapping code into executable Docker images.

Remember, you are not allowed to run any code on any of these devices outside a proper Duckietown module. So, if you have code to run, you need to put it in a module first.

```{tableofcontents}
```