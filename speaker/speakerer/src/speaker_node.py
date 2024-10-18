"""
    Jason Hughes
    September 2024
    Node that startes recording from the microphone when
    a trigger is recieved. After 10 seconds its publishes the string

    DTC PRONTO 2024
"""

import rospy
from std_msgs.msg import Bool, String, UInt8
from whisper_mic import WhisperMic

class Yapper:

  def __init__(self):
    self.trigger_ = 0
    self.volume = 1.0
    self.voice = 'voice_kal_diphone'
    self.speech = "Please remain still. We are here to help."

    rospy.Subscriber("/jackal_teleop/trigger", UInt8, self.triggerCallback)
    print("[YAPPER] Yapper Node initialized")
  def callback(self, msg : UInt8) -> None:
    self.trigger_ = msg.data
    if self.trigger_ > 0:
      # play audio
      soundhandle.say(self.speech, self.voice, self.volume)
      rospy.loginfo("[YAPPEr] Result %s" self.speech)
      print("spoken:", self.speech)

    else:
      # trigger is false (dunno if we need this)
      pass

if __name__ == "__main__":
    rospy.init_node("speaker")
    Yapper()
    rospy.spin()
