(program-duckiebot-run)=
# Run on your Duckiebot

Now that we know how to package a piece of software into a Docker image for Duckietown, we can go one step further and write code that will run on the robot instead of our laptop.

This part assumes that you have a Duckiebot up and running with hostname `MY_ROBOT`. Of course you don’t need to change the hostname to `MY_ROBOT`, just replace it with your robot name in the instructions below. You can make sure that your robot is ready by executing the command

    ping MY_ROBOT.local

If we can ping the robot, we are good to go.

Let us go back to our script file my_script.py and change it to:

```python
import os
message = "Hello from %s!" % os.environ['VEHICLE_NAME']
print(message)
```

We can now modify slightly the instructions for building the image so that the image gets built directly on the robot instead of your laptop or desktop machine. Run the command

    dts devel build -f --arch arm32v7 -H MY_ROBOT.local

As you can see, we changed two things, one is `--arch arm32v7` which tells Docker to build an image that will run on ARM architecture (which is the architecture the CPU on the robot is based on), the second is `-H MY_ROBOT.local` which tells Docker where to build the image.

Once the image is built, we can run it on the robot by running the command

    docker -H MY_ROBOT.local run -it --rm --net=host duckietown/my-program:latest-arm32v7

Please take into consideration that the image tag may be different, you can check the correct image name and tag in the end of the output the build command.
If everything worked as expected, you should see the following output,

```
The environment variable VEHICLE_NAME is not set. Using 'MY_ROBOT'.
Adding /code/my-program to PYTHONPATH
Adding /code/dt-commons to PYTHONPATH
Activating services broadcast...
Done!

Hello from MY_ROBOT!

Deactivating services broadcast...
Done!
```


Congratulations! You just built and run your first Duckietown-compliant and Duckiebot-compatible Docker image.

