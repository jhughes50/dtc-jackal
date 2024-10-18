# Whisper

This module does speech-to-text inference with the open-ai [whisper](https://github.com/openai/whisper). It is based on my fork of [whisper-mic](https://github.com/jhughes50/whisper_mic). Then a trigger is recieved it waits for the speaker message to play, then is listens for ten seconds and passes that audio signal to whisper. The text is published on topic `/whisperer/text`. 

### Running Standalone 

To run this standalone run the following:
 - start the docker image: `./run.bash`
 - start the ros node: `roslaunch whisperer whisperer.launch timeout:=10` where `timeout` is the how long the microphone will listen for.

### Configure Asound 
The microphone is plugged directly into the motherboard via usb and shows up as its own soundcard. To see what ID the microphone is associated with run `cat /proc/asound/cards` and you'll see something like the following:
```
 0 [Nano           ]: USB-Audio - Yeti Nano
                      Blue Microphones Yeti Nano at usb-0000:67:00.0-12, full speed
 1 [Audio          ]: USB-Audio - USB Audio
                      Generic USB Audio at usb-0000:67:00.0-2.2, high speed
 2 [NVidia         ]: HDA-Intel - HDA NVidia
                      HDA NVidia at 0xde080000 irq 121
 3 [Generic        ]: HDA-Intel - HD-Audio Generic
                      HD-Audio Generic at 0xde780000 irq 123
```
In this case the microphone is device `0` here. The number in `config/asound.conf` should match this number. 

### TODOs 
 - Order and test with new soundcard.
