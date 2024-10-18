#!/usr/bin/env python3
import rospy
import torch
import os
import whisper

from std_msgs.msg import UInt8
from std_msgs.msg import String
from sound_play.msg import SoundRequest
from sound_play.libsoundplay import SoundClient

device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
model = whisper.load_model("base", device) 
print("model has been loaded")

# https://wiki.ros.org/sound_play/Tutorials/ConfiguringAndUsingSpeakers
# https://github.com/ros-drivers/audio_common/tree/master/sound_play/scripts 

soundhandle = SoundClient()
rospy.sleep(1)

whisper_pub = rospy.Publisher('/whisper', String, queue_size=10)

def callback(msg : UInt8) -> None:
  trigger = msg.data
  if trigger > 0:
    # play audio
    speech = "Please remain still. We are here to help."
    voice = 'voice_kal_diphone'
    volume = 1.0
    soundhandle.say(speech, voice, volume)
    
    print("spoken:", speech)
    
    # sleep for 10 seconds
    rospy.sleep(10.)
    # TODO: test this stream audio from outside and save to a file
    os.popen('roslaunch audio_capture capture_to_file.launch device:=hw:3 dst:="~/ws/src/whisperer/audio_outout.mp3" sample_rate:=48000 format:=wave channels:=2 sample_format:=S16LE', 'r', 1)
    print("listening to audio....")
    rospy.sleep(10)

    os.popen('rosnode kill audio_capture', 'r', 1)
    rospy.sleep(2)

    # save the audio and use whisper's built in function or load in manually
    # replace Recording.m4a with name of file:
    audio = whisper.load_audio("~/ws/src/whisperer/audio_output.mp3")
    # make log-Mel spectrogram and move to the same device as the model
    mel = whisper.log_mel_spectrogram(audio).to(model.device)
    options = whisper.DecodingOptions()
    result = whisper.decode(model, mel, options)
    
    whisper_msg = String()
    whisper_msg.data = result.text
    print("detected audio:", whisper_msg)
    
    pub.publish(whisper_msg)
  else:
    # trigger is false (dunno if we need this)
    pass
  
def listener():
  rospy.init_node('audio_detection_node', anonymous=True)
  rospy.Subscriber("/jackal_teleop/trigger", UInt8, callback)
  # keeps python from exiting until this node is stopped
  rospy.spin()

if __name__ == '__main__':
    listener()
