# Speaker
This node controls the speaker based on the trigger count. When a trigger is received this plays the corresponding message for that trigger. 

### Configuring Asound
The speaker is plugged into a usb sound card with a corresponding ID number. That can be checked with `cat /proc/asound/cards` and you'll see something like:
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
The usb soundcard is device `1` here. The number in `config/asound.conf` should match this number. 

