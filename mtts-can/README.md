# MTTS-CAN ROS
This image contains a ros implementation of the MTTS-CAN network. This will output a heart rate and respiration rate on `/heart_rate/model` and `/respiration_rate/model` topics respectively. This node subscribes to the `/camera/image_color` topic. When a trigger is received, it saved frames for 10 seconds. Once it has saved frames the frames it passes them to the nework which spits out a heart rate and respiration rate signal. The signal is then processed using scipy to get a heart rate and respiration rate. 

To run this image on its own:
```
./run.bash # to start the docker image
roslaunch mtts_can_ros mtts.launch
```

