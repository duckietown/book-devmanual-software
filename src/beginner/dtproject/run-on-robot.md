(dtproject-run-on-duckiebot)=
# Run on your Duckiebot

Now that we know how to package a piece of software into a Docker image for Duckietown using DTProjects, 
we can go one step further and write code that will run on the robot instead of our local computer.

This part assumes that you have a Duckiebot up and running with hostname `ROBOT_NAME`. 
Of course, you don’t need to change the hostname to `ROBOT_NAME`, just replace it with your robot name 
in the instructions below. You can make sure that your robot is ready by executing the command

    ping ROBOT_NAME.local

If we can ping the robot, we are good to go.

Let us go back to our script file `my_script.py` we created in [](dtproject-add-your-code) and change it to:

```python
import os

vehicle_name = os.environ['VEHICLE_NAME']
message = f"\nHello from {vehicle_name}!\n"
print(message)
```

We can now modify slightly the instructions for building the image so that the image gets built directly 
on the robot instead of our computer. Run the command

    dts devel build -f -H ROBOT_NAME

As you can see, we added `-H ROBOT_NAME` to the build command. This new option tells `dts` where to 
build the image.

Once the image is built, we can run it on the robot by running the command

    dts devel run -H ROBOT_NAME

If everything worked as expected, you should see the following output,

```
...
==> Launching app...

Hello from ROBOT_NAME!

<== App terminated!
```


```{admonition} Congratulations 🎉
You just built and run your first Duckietown-compliant and Duckiebot-compatible Docker image.
We are sure this is just the first of many!
```
