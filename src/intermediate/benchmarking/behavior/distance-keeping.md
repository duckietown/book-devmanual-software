# Distance keeping

```{todo}
This is a student project, put in the attic for now.
```

Prepare two Duckiebots, one dummy and one that is used for the testing. Then set the gain of the 'dummy' 
Duckiebot to 0.6 and make sure the gain of the autobot is at 1.0. Place the two Duckiebots behind each other 
and start lane_following on both of them. Let them drive for half a round and then increase the gain of the 
'dummy' Duckiebot to 0.8. Half a round later increase the gain to 1.0.
During this time record the bags on both the Duckiebot and the localization system. For the Duckiebot please 
record the following nodes:

* /AUTOBOTNAME/vehicle_avoidance_control_node/car_cmd
* /AUTOBOTNAME/vehicle_avoidance_control_node/switch
* /AUTOBOTNAME/vehicle_avoidance_control_node/vehicle_detected
* /AUTOBOTNAME/vehicle_detection_node/detection
* /AUTOBOTNAME/vehicle_detection_node/detection_time
* /AUTOBOTNAME/vehicle_detection_node/switch
* /AUTOBOTNAME/vehicle_filter_node/switch

```{note}
The Duckiebot used as a 'dummy' that drives in front does not need to have the acquisition bridge running.
Then with the bags recorded run the notebook that analysis the engineering data, this is compatible for any 
kind of benchmark.
Within the notebook 95 add some analysis that compares the distance between the two Duckiebots to the 
reference distance the Duckiebot in the back was supposed to keep. This can be done very easily by 
comparing the relative pose of the two Duckiebots. At the same time analyze the calculation the Duckiebot 
in the back has made and compare its estimated relative distance to the ground truth measured by the 
watchtowers.
Also check how well lane following is doing when having a Duckiebot in front.
Please note that you will have to add some lines into the lane_following.launch file to start the vehicle 
detection.
```

As a metrics use: for the engineering data the same as for Lane Following, and for the benchmarking the mean 
difference between the actual distance and the reference distance, the mean difference between the estimated 
distance by the Duckiebot and the actual distance measured by the watchtowers.
