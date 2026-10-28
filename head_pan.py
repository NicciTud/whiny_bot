#!/usr/bin/env python3
import rospy
from std_msgs.msg import Bool, Float32
from intera_core_msgs.msg import HeadPanCommand

#Default and panning speed constants 
RESET = 0.0
SPEED = 0.2

def main():
    rospy.init_node("staring_state")

    # Initialise publisher and state
    pub = rospy.Publisher("/robot/head/command_head_pan", HeadPanCommand, queue_size=1)
    # State the global variable to store the current state of the panning and whether it is enabled or not
    state = {"enabled": False, "pan": RESET}


    def move(target):
        cmd = HeadPanCommand()
        cmd.target = target
        cmd.speed_ratio = SPEED
        cmd.pan_mode = HeadPanCommand.SET_ACTIVE_CANCELLATION_MODE
        pub.publish(cmd)

    # Angle callback (Rem: the angle is in radians)
    def on_angle(msg):
        state["pan"] = msg.data

    # Toggle for enabling/disabling the panning state
    def on_flag(msg):
        previous_state = state["enabled"]
        state["enabled"] = msg.data
        if previous_state and  msg.data == False:
            move(RESET)


    # Initialise the subscribers 
    rospy.Subscriber("looking_at_person/enable", Bool, on_flag, queue_size=1)
    rospy.Subscriber("looking_at_person/panning_angle", Float32, on_angle, queue_size=1)

    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        if state["enabled"]:
            move(state["pan"])
        rate.sleep()


if __name__ == "__main__":
    main()