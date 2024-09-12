"""
    Jason Hughes
    September 2024
    Node that startes recording from the microphone when
    a trigger is recieved. After 10 seconds its publishes the string

    DTC PRONTO 2024
"""

import rospy
from std_msgs.msg import Bool, String
from whisper_mic import WhisperMic

class LiteralListener:

    def __init__(self):
        root = rospy.get_param("/whisperer/model_root")
        model = rospy.get_param("whisperer/model")

        self.mic_ = WhisperMic(model=model, english=True, model_root=root)
        self.timeout_ = rospy.get_param("/whisperer/timeout", default=10)

        rospy.Subscriber("/jackal_teleop/trigger", Bool, self.triggerCallback)
        self.text_pub_ = rospy.Publisher("/text", String, queue_size=4)
        print("[WHISPERER] Whisperer Node initialized")

    def triggerCallback(self, msg : Bool) -> None:
        if msg.data:
            print("[WHISPERER][STATUS] Trigger Recieved")
            result = self.mic_.listen(timeout=self.timeout_) # blocking function

            text_msg = String()
            text_msg.data = result

            self.text_pub_.publish(text_msg)

if __name__ == "__main__":
    rospy.init_node("whisperer")
    LiteralListener()
    rospy.spin()
