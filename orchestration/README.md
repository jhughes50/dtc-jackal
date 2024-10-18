# Orchestration
The orchestration node is what organizes all the inference readings and sends them out when they are all aggregated. It also takes care of saving and sending periodic images. Finally, `MOCHA` and the apriltag detection should be run in this container. 

### How it works
The orchestration node first reads the `docker-compose.yml` file to see what inference components are running, these are set to a `running=true`, meaning the orchestrator will wait for the reading from that inference module until sending out the readings. When a trigger is recieved, the `recieved` state will turn to false. Once the reading is recieved from a module the recieved state will turn to `true`. The orchestration will periodically output statuses of containing the trigger count `{1,2,3}`, the `running` state and the `recieved`. If a module is not running it will say that module is `not running`. If a module is running but has not recieved data it will say `not ready` in red. Once a reading is received for the at module the status will say `ready` in green. Additionally, when a trigger is received, an image is downresed and saved every 5 seconds for 10 seconds for a total of three image. These iamges are then compressed and aggregated into one topic and published on the `/ground_image` topic. The inference readings are published on the `/ground_detection` topic.

### Running Standalone
If you want to run the orchestration node as standalone (which I do not recommend) do the following:
```
./run.bash
roslaunch gone orchestrator.launch
```
This will startup the orchestrator, a ros monitor and the apriltag detector.

### Ground Orchestration Node (gone)
This is the ros orchestration package. 
__Custom Msgs__
 - `GroundDetection.msg`: Contains structures for a 
    - `header`: contains time stamp `stamp` and `frame_id` which should be the name of the robot. 
    - `gps`: for latitude and longitude
    - `casualty_id`: april tag id reading
    - `whisper`: string of text from whisper output
    - `acconeer_respiration_rate`: respiration rate from radar
    - `event_respiration_rate`: intended to report the respiration rate reading from the event camera, but is currently reports the respiration rate from the MTTS-CAN network.
    - `nueral_heart_rate`: reports the heart rate from MTTS-CAN
    - `cv_heart_rate`: reports the heart rate from pyvhr
 - `GroundImage.msg`: contains three periodic images
    - `header`: time stamp and frame id
    - `gps`: for latitude and longitude
    - `image1`: downresed and compressed rgb image at time 0.
    - `image2`: downresed and compressed rgb image at time 1.
    - `image3`: downresed and compressed rgb image at time 3.

__ROS Monitor__
This package also contains a ros monitor, this will detect active ros nodes and notify you when nodes die or when new nodes are added to the system. If a node crashes for any reason in anycontainer it will report which node died so you can hopefully restart it.

### Apriltag ROS
`Apriltag_ROS` is installed in this docker image. You can reconfigure which tags it will be able to recognize in `config/tags.yaml`. This is persistent file.

### TODOS
 - Robot name is hardcoded, we should be able to get this from the `hostname` environement varaible.
 - Make the sysout change what is there rather than append to terminal, this will make it much more readable during the competition.

