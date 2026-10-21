#!/usr/bin/env python

import sys
import rospy
import moveit_commander
from sensor_msgs.msg import JointState

rospy.init_node('sawyer_emotes', anonymous=False)

mgc_global = moveit_commander.MoveGroupCommander("arm")
mgc_global.set_max_velocity_scaling_factor(1.0)
mgc_global.set_max_acceleration_scaling_factor(1.0)

# neutral pose, similar to ready
BASE = [0.758423670034864, 0.16016586314455136, 0.16016586314455136, 
        -0.04326329491007631, 0.9166717183780757, 0.00042531220077697416, 0.39729216849674026]

def go(positions):
    js = JointState()
    js.name = ["right_j0", "right_j1", "right_j2", "right_j3", "right_j4", "right_j5", "right_j6"]
    js.position = positions
    mgc_global.set_joint_value_target(js)
    mgc_global.go(wait=True)

def wiggle():
    go(BASE)
    go([-0.05, -0.83, -0.11, 2.09, 0.30, 0.20, -0.10])
    go(BASE)
    go([-0.05, -0.83, -0.11, 2.09, 0.05, 0.45, -0.30])
    go(BASE)

def nod():
    go(BASE)
    go([-0.05, -0.83, -0.11, 2.00, 0.17, 0.33, -0.21])
    go(BASE)

def wave():
    go(BASE)
    go([-0.05, -0.60, -0.11, 1.80, 0.40, 0.10, -0.40])
    go([-0.05, -0.60, -0.11, 1.80, 0.00, 0.50, 0.10])
    go([-0.05, -0.60, -0.11, 1.80, 0.40, 0.10, -0.40])
    go(BASE)

def sad():
    go(BASE)
    go([-0.05, -0.95, -0.15, 1.75, 0.10, 0.55, -0.35])
    rospy.sleep(rospy.Duration(1.5))
    go(BASE)

def shake():
    go(BASE)
    go([-0.05, -0.83, -0.11, 2.09, 0.35, 0.30, -0.30])
    go([-0.05, -0.83, -0.11, 2.09, -0.05, 0.35, -0.10])
    go([-0.05, -0.83, -0.11, 2.09, 0.35, 0.30, -0.30])
    go(BASE)

if __name__ == "__main__":
    choice = sys.argv[1] if len(sys.argv) > 1 else "wiggle"
    if choice == "wiggle":
        wiggle()
    elif choice == "nod":
        nod()
    elif choice == "wave":
        wave()
    elif choice == "sad":
        sad()
    elif choice == "shake":
        shake()
    else:
        print("options: wiggle nod wave sad shake")