"""
    Jason Hughes
    September 2024
    Node that startes recording from the microphone when
    a trigger is recieved. After 10 seconds its publishes the string

    DTC PRONTO 2024
"""

import rospy
from std_msgs.msg import Bool, String, UInt8
from sound_play.msg import SoundRequest
from sound_play.libsoundplay import SoundClient

class LiteralListener:

    def __init__(self):
        self.trigger_ = 0

        self.soundhandle_ = SoundClient()

        rospy.Subscriber("/jackal_teleop/trigger", UInt8, self.triggerCallback)
        print("[SPEAKER][STATUS] Speaker Node initialized")

    def triggerCallback(self, msg : UInt8) -> None:
        self.trigger_ = msg.data
        if self.trigger_ == 0:
            
            speech = "Please remain calm. We are here to help."
            voice = 'voice_kal_diphone'
            volume = 1.0
            self.soundhandle_.say(speech, voice, volume)
            print("[SPEAKER] Sent Sound")
        elif self.trigger_ == 1:
            print("[SPEAKER][STATUS] Trigger Recieved")

            # speak
            speech = "Please look at the camera and then remain still. We are assessing your injuries"
            voice = 'voice_kal_diphone'
            volume = 1.0
            self.soundhandle_.say(speech, voice, volume)

        elif self.trigger_ == 2:
            print("[SPEAKER][STATUS] Trigger Recieved")

            # speak
            speech = "Please waave at the robot and tell us where you are injured"
            voice = 'voice_kal_diphone'
            volume = 1.0
            self.soundhandle_.say(speech, voice, volume)


if __name__ == "__main__":
    rospy.init_node("speaker")
    LiteralListener()
    rospy.spin()
