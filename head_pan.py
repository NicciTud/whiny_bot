#!/usr/bin/env python3
import rospy
from std_msgs.msg import Bool, Float32
from intera_core_msgs.msg import HeadPanCommand

#Default and panning speed constants 
RESET = 0.0
DEFAULT_SPEED = 0.2
def main():
    rospy.init_node("staring_state")

    # Initialise publisher and state
    pub = rospy.Publisher("/robot/head/command_head_pan", HeadPanCommand, queue_size=1)
    # State the global variable to store the current state of the panning and whether it is enabled or not
    state = {"enabled": False, "pan": RESET, "speed": DEFAULT_SPEED}


    def move(target):
        cmd = HeadPanCommand()
        cmd.target = target
        cmd.speed_ratio = state["speed"]
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

    #callback for the speed
    def on_speed(msg):
        # speed ratio should be between 0.0 and 1.0
        state["speed"] = max(0.0, min(1.0, msg.data))


    # Initialise the subscribers 
    rospy.Subscriber("looking_at_person/enable", Bool, on_flag, queue_size=1)
    rospy.Subscriber("looking_at_person/panning_angle", Float32, on_angle, queue_size=1)
    rospy.Subscriber("looking_at_person/speed", Float32, on_speed, queue_size=1)

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        if state["enabled"]:
            move(state["pan"])
        rate.sleep()


if __name__ == "__main__":
    main()