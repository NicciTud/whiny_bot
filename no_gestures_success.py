#!/usr/bin/env python

import sys
import rospy
import moveit_commander
import geometry_msgs.msg
from copy import deepcopy
from control_msgs.msg import GripperCommand
from sensor_msgs.msg import JointState
from std_msgs.msg import String as StringMsg
import random

'''
IN PROGRESS check in lab
pose_z - DESCEND >= tabletop_z , don't go below the table top.
pose_z - DESCEND <= tabletop_z + h, you're at/below the block top, so you're actually on the block, not above it.
'''

DESCEND   = 0.185   # approach to grasp descent, tuned so gripper closes on block
RETREAT_Z = 0.30   # post-drop lift to clear box
RANDOMIZE = True  # True for actual, False for debugging
gap = 0.12    # y step between blocks in the same row, 6 cm apart center to center

rospy.init_node('no_gestures_pickup', anonymous=False)

#gripper commands, open close
gripper_cmd = rospy.Publisher('/robot/gripper/command', GripperCommand, queue_size=1)
def open_gripper():
    cmd = GripperCommand()
    cmd.position = 1.0
    cmd.max_effort = 1.0
    gripper_cmd.publish(cmd)
    rospy.sleep(0.5)
def close_gripper():
    cmd = GripperCommand()
    cmd.position = 0.0
    cmd.max_effort = 1.0
    gripper_cmd.publish(cmd)
    rospy.sleep(0.5)

    #user prompts, can remove
say_pub = rospy.Publisher('/say', StringMsg, queue_size=1)
def say(text):
    msg = StringMsg()
    msg.data = text
    say_pub.publish(msg)


mgc_global = moveit_commander.MoveGroupCommander("arm")
mgc_global.set_max_velocity_scaling_factor(1.0)
mgc_global.set_max_acceleration_scaling_factor(1.0)
mgc = moveit_commander.MoveGroupCommander("arm_local")
mgc.set_max_velocity_scaling_factor(1.0)
mgc.set_max_acceleration_scaling_factor(1.0)

psi= moveit_commander.PlanningSceneInterface()

'''
IN PROGRESS: adjust table
'''
table_pose = geometry_msgs.msg.PoseStamped()
table_pose.header.frame_id = "base"
table_pose.pose.position.x = 0.75 #maybe a few cm forward so we dont get joint limited
table_pose.pose.position.y = -0.13
table_pose.pose.position.z = -0.28
table_pose.pose.orientation.w = 1.0 #check, make sure it's staight
psi.add_box("table", pose=table_pose, size=(0.8, 0.8, 0.10)) #edit actual table size

window_collider_pose = geometry_msgs.msg.PoseStamped()
window_collider_pose.header.frame_id = "base"
window_collider_pose.pose.position.x = 0.3
window_collider_pose.pose.position.y = 0.6
window_collider_pose.pose.position.z = 0.0
window_collider_pose.pose.orientation.w = 1.0
psi.add_box("window_collider", pose=window_collider_pose, size=(1.6, 0.2, 1.5))

'''
IN PROGRESS
create reference joint states for box to drop cubes into
'''
#drop into box pose
box_posture = JointState()
box_posture.name = ["right_j0", "right_j1", "right_j2", "right_j3", "right_j4", "right_j5", "right_j6"]
box_posture.position = [-0.2729189453125, 0.3767265625, -0.222205078125, -0.7939072265625, 0.250521484375, 1.6111279296875, 0.026794921875]


#positions dict and keys
'''
IN PROGRESS
update with all 0-6 initial block positions in lab
make sure it is positioned a bit above the block, so it can crane game descend
'''

#hardcoded in lab

GRASP_QUAT = (0.07169260225135074,-0.9963911848868929, 0.015748911465860584,0.04262334160720189)   #pls check, find from top grasp

anchor_even = geometry_msgs.msg.PoseStamped()
anchor_even.header.frame_id = "base"
anchor_even.pose.position.x = 0.65       #check pls, back row
anchor_even.pose.position.y = -0.3       #even rows start coordinate
anchor_even.pose.position.z = 0           #clearance above block

anchor_even.pose.orientation.x = GRASP_QUAT[0]
anchor_even.pose.orientation.y = GRASP_QUAT[1]
anchor_even.pose.orientation.z = GRASP_QUAT[2]
anchor_even.pose.orientation.w = GRASP_QUAT[3]

anchor_odd = deepcopy(anchor_even)
anchor_odd.pose.position.x = 0.75        #check pls, 10 cm ahead of back row
anchor_odd.pose.position.y = -0.25       #odd start, 5 cm off

positions = {}
for i in range(7):
    base = anchor_even if i % 2 == 0 else anchor_odd
    loc = deepcopy(base)
    loc.pose.position.y = base.pose.position.y + (i // 2) * gap
    positions["block_%d" % i] = loc


def move_slow(mgc):
    mgc.set_max_velocity_scaling_factor(0.03)
    mgc.set_max_acceleration_scaling_factor(0.1)

def move_fast(mgc):
    mgc.set_max_velocity_scaling_factor(1.0)
    mgc.set_max_acceleration_scaling_factor(1.0)

def move_to_box():
    mgc_global.set_joint_value_target(box_posture)
    return mgc_global.go(wait=True)

def move_to_pose(pose):
    mgc.set_pose_target(pose)
    return mgc.go(wait=True)

def ready():
    mgc_global.set_named_target("ready")
    mgc_global.go(wait=True)

#for picking up cube, literally just move and grasp

def pick_up_cube(key):
    open_gripper()
    move_fast(mgc)
    if not move_to_pose(positions[key]):
        return False

    #lower
    grasp = deepcopy(positions[key])
    grasp.pose.position.z -= DESCEND
    move_slow(mgc)
    if not move_to_pose(grasp):
        return False

    #close
    close_gripper()

    #go up
    move_fast(mgc)
    if not move_to_pose(positions[key]):
        return False

    return True


# going to position of block and executing pick_up_cube()
def get_block_in_line(key):
    if not pick_up_cube(key):
        return False

    #move to box, and drop
    if not move_to_box():
        return False
    descend = deepcopy(mgc.get_current_pose())
    descend.pose.position.z -= 0.1
    if not move_to_pose(descend):
            return False
    open_gripper()
    rospy.sleep(rospy.Duration(1.0))

    #move up so it doesnt hit box
    retreat = deepcopy(mgc.get_current_pose())
    retreat.pose.position.z += RETREAT_Z
    move_fast(mgc)
    if not move_to_pose(retreat):
        return False

    return True

#actual start
'''
IN PROGRESS, random selector of initial positions
'''
if RANDOMIZE: #when randomize active, check top
    todo = list(positions.keys())
    random.shuffle(todo)
else:
    todo = list(positions.keys())

print("pick order: ", todo)

#actual pick up and drop into box routine
for key in todo:
    ready()
    rospy.sleep(rospy.Duration(1.0))
    get_block_in_line(key)
    rospy.sleep(rospy.Duration(1.0))
ready()


'''
#for testing:
key = "block_0" #or for key in positions
ready()
rospy.sleep(rospy.Duration(1.0))
ok = get_block_in_line(key)
rospy.loginfo("block %s: %s" % (key, "OK" if ok else "FAILED"))
rospy.sleep(rospy.Duration(1.0))
ready()
'''