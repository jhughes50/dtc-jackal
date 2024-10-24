# The DTC Jackal Stack
This is the ultimate guide to running the dtc jackal.

### Powering On and Off
To power on the jackals short the second and third pins from the left. The jackals should be powered off from the command line. Run `sudo shutdown now`. This will power off the entire jackal.

### Logging In
There are thre main ways to log into the jackal. If you are indoors the jackal will automatically connect to the `mrsl_perch` network with the following static ips:
 - Phobos: `192.168.129.111`
 - Deimos: `192.168.129.112`

If you go outdoors you can use the `jackalnet2` hotspot, the robots will automatically connect to it only if they are out of range with `mrsl_perch` with the following static ips:
 - Phobos: `192.168.50.111`
 - Deimos: `192.168.50.112`

Finally, you may connect over the rajant network. The jackals will automatically connect to the rajant no matter what but note that the rajants take a few minutes to boot. To connect via rajant, use a rajant brick and connect to via ethernet. Assign yourself and a static ip address in the network configuration window on linux. Give yourself an ip address of `10.10.10.X` where X is in the range of `0 - 100` (over 100 is reserved for the robots), and a netmask of `255.0.0.0`. I strongly recommoned not using a windows computer for this. On the rajant network the jackals will have the following static ips:
 - Phobos: `10.10.10.111`
 - Deimos: `10.10.10.112`

Once you have network configurations correct on your ground station you can ssh into the jackal with: `ssh dtc@<jackal_ip>`, the password is a space.

### Running the Jackal
Once you have logged into the jackal, navigate to the `Docker` directory with `cd Docker`. Here you can configure what components you want to run in the `docker-compose.yml`. In the `jackal-base` image you can configure what sensors you want to run, by setting the environment variable for the respective sensor to `true/false`. Setting the value to true will start the ros device driver, otherwise it will the driver will not be running and you will not see the rostopics. Additionally, in `docker-compose.yml` you can configure what inference components you want to run in the respective image by setting the `RUN` variable to `true/false`. Once you have your `docker-compose.yml` configured you can start the jackal stack with the following steps.
 - `tmux`
 - `docker compose up`
The system is now running and the jackal will be driveable.

### Inference Components
Each of the inference components run in their own respective docker images. More detail on what each one of them does is in their respective readme's.
 1. `acconeer`: Takes care of radar device driver and radar inferenceing.
 2. `data`: This is not an inference component. Each docker image has a persistent directory called `data` which gets mapped to this data folder outside of docker. Any data or file that is outside of the data directory in docker will be deleted when docker is shutdown. Read more on persistent volumes [here](https://docs.docker.com/engine/storage/volumes/).
 3. `jackal`: This module houses all the ros device drivers for the gps, event camera rgb camera, boson and lidar. This also takes care of drivers for the controller and the jackal itself. This needs to be started in order to drive the jackal.
 4. `masking`: Contains a yolov8 pyotch implementation that masks and crops a person if one is detected.
 5. `mtts-can`: Contains a tensorflow implementation of [MTTS](https://github.com/xliucs/MTTS-CAN) which is network for heart rate and respiration rate. 
 6. `orchestration`: This component listens to all the infernce topics that are __active__. If and inference module is not started in the docker-compose yaml then the orchestrator will ignore it. Once a trigger has been initialized from the controller, this will wait until all active components have finished their inferencing. Once it receives all the readings it will send the inference readings on a rostopic. 
 7. `pyvhr`: This contains a computer vision approach to heart rate detection
 8. `speaker`: This controls the speaker.
 9. `whisper`: This contains an implementation of [whisper-mic](https://github.com/mallorbc/whisper_mic) for speech to text. 

### Running MOCHA
During the competition all communication is done through [MOCHA](https://github.com/KumarRobotics/MOCHA). This should be launched seperately from everything else after `docker compose up`. MOHCA is installed in the `orchestration` container. Once you've started everything with docker compose go to the orchestration folder and join the docker container, then launch mocha.
```
cd orchestration
./join.bash
roslaunch mocha_launch jackal.launch robot_name:=<phobos,deimos>
```

### Bagging Data
If you want to bag data, follow the start up instruction above. Once your started the jackal with the desired sensors and inference componenets, enter the `jackal-base` image. If you started in `tmux` split your window with `ctrl-b + %` or `ctrl-b + "`, then `cd jackal`. Enter the `jackal-base` image with `./join.bash`. You can also open a new terminal on your groundstation, ssh into the jackal navigate to the jackal directory and run `./join.bash` there. Once in `jackal-base` docker image run `cd data`. This is the persistent data directory and is where all rosbags should be saved. Now you can run `rosbag record <list_of_topics>` to start bagging. If you are bagging camera data bag the `<topic_name>/compressed`, additionally, do not bag event camera data for too long. If you want to bag lidar data, bag the `lidar_packets` and `imu_packets` rather than the point clouds.

Once you have collected your bag file, you can copy it over to your machine. If the file is not too large you can copy it over wifi with:
``` 
scp /path/to/<date_time>.bag <your_username>@<your_ip_address>:/path/to/<where_you_want_to_save_file>
```
If you files are large it will be faster to use one of the portable SSDs. __Note__: you can only use one of the dtc encrypted drives if you have a linux machine to copy it to. Plug the drive into the robot via usb. Run:
```
lsblk # to see the usb device name, usually sda1 or sdb1
sudo cryptsetup luksOpen /dev/sda1 samsung_t7 # unencrypt
sudo mount /dev/mapper/samsung_t7 /media/dtc # mount
sudo cp /path/to/<bag_file>.bag /media/dtc/ #copies the files
sudo umount /media/dtc # unmount when your done copying data
sudo cryptsetup luksClose samsung_t7
```
Now the portable ssd can be safely unplugged. 

### Notes
Note that pytorch or tensorflow models are not included in this repo, neither are the install files for some of the jackal device drivers. This will have to be copied over manually.  
