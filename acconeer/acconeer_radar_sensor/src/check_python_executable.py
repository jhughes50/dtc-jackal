# check_python_executable.py
import rospy
import sys

def check_python():
    rospy.init_node('check_python_executable', anonymous=True)
    python_executable = sys.executable
    rospy.loginfo(f"ROS is using this Python executable: {python_executable}")

if __name__ == '__main__':
    try:
        check_python()
    except rospy.ROSInterruptException:
        pass
