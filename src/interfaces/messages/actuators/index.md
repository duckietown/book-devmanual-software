# Messages - `actuators`

These messages are used to carry information about the state of the actuators in the system. 
Examples of actuators are motors, LEDs, and Display.

## AttitudePIDParameters

Message providing the gains for the attitude (Roll, Yaw, Pitch) PIDs of a flight controller.

```{eval-rst}
.. autopydantic_model:: duckietown_messages.actuators.attitude_pids_parameters.AttitudePIDParameters
    :member-order: bysource
    :inherited-members: pydantic.BaseModel
```

## DifferentialPWM

This message is used to describe the speed of the left and right wheels of a differential drive robot.

```{note}
This message can be used both as an input (control the robot) and as an output (report the robot's state).
```

```{eval-rst}
.. autopydantic_model:: duckietown_messages.actuators.differential_pwm.DifferentialPWM
    :member-order: bysource
    :inherited-members: pydantic.BaseModel
```

## DroneControl

Control message for the drone, containing an attitude and throttle setpoint.

```{eval-rst}
.. autopydantic_model:: duckietown_messages.actuators.drone_control.DroneControl
    :member-order: bysource
    :inherited-members: pydantic.BaseModel
```

## DroneMode

Message and response message for the trasnform changing the mode of the drone.


```{eval-rst}
.. autopydantic_model:: duckietown_messages.actuators.drone_mode.DroneModeMsg
    :member-order: bysource
    :inherited-members: pydantic.BaseModel
```

```{eval-rst}
.. autopydantic_model:: duckietown_messages.actuators.drone_mode.DroneModeResponse
    :member-order: bysource
    :inherited-members: pydantic.BaseModel
```

`mode` can be one of the following:

```{eval-rst}
.. autopydantic_model:: duckietown_messages.actuators.drone_mode.Mode
    :member-order: bysource
    :inherited-members: pydantic.BaseModel
```

## DroneMotorCommand

This message contains the motor PWM commands and information on the range `[Minimum, Maximum]`.
```{eval-rst}
.. autopydantic_model:: duckietown_messages.actuators.drone_motor_command.DroneMotorCommand
    :member-order: bysource
    :inherited-members: pydantic.BaseModel
```

## CarLights

This message is used to describe the state of the lights of a Duckiebot.

```{note}
This message can be used both as an input (control the lights) and as an output (report the robot's state).
```

```{eval-rst}
.. autopydantic_model:: duckietown_messages.actuators.car_lights.CarLights
    :member-order: bysource
    :inherited-members: pydantic.BaseModel
```


## DisplayFragment

Display fragments are 2D binary images that can be displayed on the screen. Fragments can be used to display text, 
icons, or other images. The overall content of the screen is defined by a list of fragments pasted in various locations
and visible at the same time.
This message is used to describe a fragment that can be displayed on a Duckiebot's screen.

```{eval-rst}
.. autopydantic_model:: duckietown_messages.actuators.display_fragment.DisplayFragment
    :member-order: bysource
    :inherited-members: pydantic.BaseModel
```


## DisplayFragments

Display fragments can be grouped together and sent to the screen driver as a single message. This message is used to
do just that.

```{eval-rst}
.. autopydantic_model:: duckietown_messages.actuators.display_fragments.DisplayFragments
    :member-order: bysource
    :inherited-members: pydantic.BaseModel
```