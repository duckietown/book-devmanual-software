# Messages - `calibrations`

This package contains messages that are used to represent calibration data for sensors and actuators.


## CameraExtrinsic

This message holds a collection of homographies that describe the transformation of pixels from the camera frame
to other reference frames. For example, the transformation from the camera frame to the ground plane frame, used 
to project detected colored segments onto the ground in front of the robot, is stored as the pair 
`("/ROBOT_NAME/base_footprint", Homography(...))`.

```{eval-rst}
.. autopydantic_model:: duckietown_messages.calibrations.camera_extrinsic.CameraExtrinsicCalibration
    :member-order: bysource
    :inherited-members: pydantic.BaseModel
```


## CameraIntrinsic

This message holds the intrinsic calibration parameters of a camera. These parameters are used to project 3D points
onto the image plane and to undistort images.

```{eval-rst}
.. autopydantic_model:: duckietown_messages.calibrations.camera_intrinsic.CameraIntrinsicCalibration
    :member-order: bysource
    :inherited-members: pydantic.BaseModel
```


## DCMotor

This message holds the calibration profile for a DC motor. A correction factor is applied to the PWM signal to account
for the non-linear relationship between the PWM signal and the motor's angular velocity. The calibration profile is
stored as a list of `(PWM, correction)` pairs for various samples of PWM values. The correction factor is a multiplier
that is applied to the PWM signal to obtain the corrected PWM signal.

```{eval-rst}
.. autopydantic_model:: duckietown_messages.calibrations.dc_motor.DCMotorCalibration
    :member-order: bysource
    :inherited-members: pydantic.BaseModel
```

