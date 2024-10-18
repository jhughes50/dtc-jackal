# Acconeer

### Setup
```
git clone https://github.com/acconeer/acconeer-python-exploration.git
python -m pip install --upgrade acconeer-exptool[app]
python -m acconeer.exptool.setup
#choose option 0 Linux for Ubuntu 20
sudo apt-get install libxcb-cursor0
python -m acconeer.exptool.app
```
### Running
To run the rosnode run: `rosrun acconeer_radar_sensor respiration_radar_sensor.py --serial-port /dev/serial/by-id/usb-Silicon_Labs_Acconeer_<UID>-if00-port0`
You can see the devices uid with `ls /dev/serial/by-id/`.

### Useful Files
 - `acconeer_radar_sensor`: This is the ros package for the acconeer radar sensor.
 - `udev`: a udev rule for the acconeer, this is not necessary to install.
 - `run.bash`: Run this file with `./run.bash` to run the acconeer docker image by itself.
