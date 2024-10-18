"""
    Jason Hughes
    September 2024
    Node that startes recording from the microphone when
    a trigger is recieved. After 10 seconds its publishes the string

    DTC PRONTO 2024
"""

import rospy
from std_msgs.msg import Bool, String, UInt8
import speech_recognition as sr
from whisper_mic import WhisperMic
from sound_play.msg import SoundRequest
from sound_play.libsoundplay import SoundClient

class LiteralListener:

    def __init__(self):
        self.root = rospy.get_param("/whisperer/model_root")
        self.model = rospy.get_param("whisperer/model")

        self.trigger_ = 0

        #self.soundhandle_ = SoundClient()

        self.mic_ = WhisperMic(model=self.model, english=True, model_root=self.root)#, mic_index=9)
        #self.recognizer_ = sr.Recognizer()
        self.timeout_ = rospy.get_param("/whisperer/timeout", default=15)

        rospy.Subscriber("/jackal_teleop/trigger", UInt8, self.triggerCallback)
        self.text_pub_ = rospy.Publisher("~text", String, queue_size=4)
        print("[WHISPERER] Whisperer Node initialized")

    def triggerCallback(self, msg : UInt8) -> None:
        self.trigger_ = msg.data
        if self.trigger_ == 0:
            pass 
            #speech = "We are here to help."
            #voice = 'voice_kal_diphone'
            #volume = 1.0
            #self.soundhandle_.say(speech, voice, volume)

        elif self.trigger_ == 1:
            print("[WHISPERER][STATUS] Trigger Recieved")

            # speak
            #speech = "Please remain still. Taking your Measurments"
            #voice = 'voice_kal_diphone'
            #volume = 1.0
            #self.soundhandle_.say(speech, voice, volume)
            #mic = WhisperMic(model=self.model, english=True, model_root=self.root)#, mic_index=9)
            rospy.sleep(rospy.Duration(4))

            # listen
            result = self.mic_.record_once(duration=self.timeout_, offset=None) # blocking function
            print("[WHISPER][STATUS] Got Result")
            #try:
            #    with sr.Microphone() as source:
            #        self.recognizer_.adjust_for_ambient_noise(source, duration=1)
            #        recorded_audio = self.recognizer_.record(source, duration=10)
            #        result = self.recognizer_.recognize_google(recorded_audio, language="en-US")
            #        rospy.loginfo("[WHISPER] Result")
            #except Exception as e:
            #    result = " "
            #    rospy.loginfo("[WHISPERER] Didn't recieve whisper result: %s" %e)
            text_msg = String()
            text_msg.data = result

            self.text_pub_.publish(text_msg)

        elif self.trigger_ == 2:
            print("[WHISPERER][STATUS] Trigger Recieved")

            # speak
            #speech = "Move if you can"
            #voice = 'voice_kal_diphone'
            #volume = 1.0
            #self.soundhandle_.say(speech, voice, volume)
            #self.mic_ = WhisperMic(model=self.model, english=True, model_root=self.root)#, mic_index=9)
            rospy.sleep(rospy.Duration(4))

            # listen
            result = self.mic_.record_once(duration = self.timeout_, offset=None) # blocking function
            print("[WHISPER][STATUS] Got Result")
            #try:
            #    with sr.Microphone() as source:
            #        self.recognizer_.adjust_for_ambient_noise(source, duration=1)
            #        recorded_audio = self.recognizer_.record(source, duration=10)
            #        result = self.recognizer_.recognize_google(recorded_audio, language="en-US")
            #        rospy.loginfo("[WHISPER] Result")
            #except Exception as e:
            #    result = " "
            #    rospy.loginfo("[WHISPERER] Didn't recieve whisper result: %s" %e)
            text_msg = String()
            text_msg.data = result

            self.text_pub_.publish(text_msg)

if __name__ == "__main__":
    rospy.init_node("whisperer")
    LiteralListener()
    rospy.spin()
