#!/usr/bin/env python
'''
Has failure conditions, for testing

'''
import sys
import rospy
import moveit_commander
import geometry_msgs.msg
from copy import deepcopy
from control_msgs.msg import GripperCommand
from sensor_msgs.msg import JointState
from std_msgs.msg import String as StringMsg
import random

DESCEND   = 0.182   # approach to grasp descent, tuned so gripper closes on block
RETREAT_Z = 0.30   # post-drop lift to clear box
RANDOMIZE = True   #True for actual, False for debugging
FAIL_COUNT = 2          #how many MAX blocks fail per run
FAIL_RATE  = 0.5        #probability a block fails *given* it's chosen
FAIL_MODES = ["repeated_drop", "empty_pickup", "drop_in_space"]

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
mgc.set_max_velocity_scaling_factor(0.2)
mgc.set_max_acceleration_scaling_factor(0.2)

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

#drop into box pose
box_posture = JointState()
box_posture.name = ["right_j0", "right_j1", "right_j2", "right_j3", "right_j4", "right_j5", "right_j6"]
box_posture.position = [-0.2729189453125, 0.3767265625, -0.222205078125, -0.7939072265625, 0.250521484375, 1.6111279296875, 0.026794921875]


#positions dict and keys, hardcoded in lab
GRASP_QUAT = (0.07169260225135074,-0.9963911848868929, 0.015748911465860584,0.04262334160720189)   #pls check, find from top grasp

positions = {}

#back row, 0
key = "block_0"
loc = geometry_msgs.msg.PoseStamped()
loc.header.frame_id = "base"
loc.pose.position.x = 0.65       #check pls, back row
loc.pose.position.y = -0.3       #even rows start coordinate
loc.pose.position.z = 0           #clearance above block

loc.pose.orientation.x = GRASP_QUAT[0]
loc.pose.orientation.y = GRASP_QUAT[1]
loc.pose.orientation.z = GRASP_QUAT[2]
loc.pose.orientation.w = GRASP_QUAT[3]
positions[key] = loc

#front row, 1
key = "block_1"
loc = geometry_msgs.msg.PoseStamped()
loc.header.frame_id = "base"
loc.pose.position.x = 0.75     
loc.pose.position.y = -0.25      
loc.pose.position.z = 0           

loc.pose.orientation.x = GRASP_QUAT[0]
loc.pose.orientation.y = GRASP_QUAT[1]
loc.pose.orientation.z = GRASP_QUAT[2]
loc.pose.orientation.w = GRASP_QUAT[3]
positions[key] = loc

#back row, 2
key = "block_2"
loc = geometry_msgs.msg.PoseStamped()
loc.header.frame_id = "base"
loc.pose.position.x = 0.65       #check pls, back row
loc.pose.position.y = -0.18       #even rows start coordinate
loc.pose.position.z = 0           #clearance above block

loc.pose.orientation.x = GRASP_QUAT[0]
loc.pose.orientation.y = GRASP_QUAT[1]
loc.pose.orientation.z = GRASP_QUAT[2]
loc.pose.orientation.w = GRASP_QUAT[3]
positions[key] = loc

#front row, 3
key = "block_3"
loc = geometry_msgs.msg.PoseStamped()
loc.header.frame_id = "base"
loc.pose.position.x = 0.75     
loc.pose.position.y = -0.13      
loc.pose.position.z = 0           

loc.pose.orientation.x = GRASP_QUAT[0]
loc.pose.orientation.y = GRASP_QUAT[1]
loc.pose.orientation.z = GRASP_QUAT[2]
loc.pose.orientation.w = GRASP_QUAT[3]
positions[key] = loc

#back row, 4
key = "block_4"
loc = geometry_msgs.msg.PoseStamped()
loc.header.frame_id = "base"
loc.pose.position.x = 0.65     
loc.pose.position.y = -0.06     
loc.pose.position.z = 0           

loc.pose.orientation.x = GRASP_QUAT[0]
loc.pose.orientation.y = GRASP_QUAT[1]
loc.pose.orientation.z = GRASP_QUAT[2]
loc.pose.orientation.w = GRASP_QUAT[3]
positions[key] = loc

#front row, 5
key = "block_5"
loc = geometry_msgs.msg.PoseStamped()
loc.header.frame_id = "base"
loc.pose.position.x = 0.75     
loc.pose.position.y = -0.01     
loc.pose.position.z = 0           

loc.pose.orientation.x = GRASP_QUAT[0]
loc.pose.orientation.y = GRASP_QUAT[1]
loc.pose.orientation.z = GRASP_QUAT[2]
loc.pose.orientation.w = GRASP_QUAT[3]
positions[key] = loc

#back row, 6
key = "block_6"
loc = geometry_msgs.msg.PoseStamped()
loc.header.frame_id = "base"
loc.pose.position.x = 0.65       #check pls, back row
loc.pose.position.y = 0.06       #even rows start coordinate
loc.pose.position.z = 0           #clearance above block

loc.pose.orientation.x = GRASP_QUAT[0]
loc.pose.orientation.y = GRASP_QUAT[1]
loc.pose.orientation.z = GRASP_QUAT[2]
loc.pose.orientation.w = GRASP_QUAT[3]
positions[key] = loc


'''
Failure conditions 
'''
#for failure condition: empty pickup
empty = deepcopy(positions["block_0"])
empty.pose.position.x = 0.85         #check pls, should be clear of both rows
empty.pose.position.y = -0.3         #y coor with no block
positions["empty"] = empty

# free-space drop pose (check pls, anywhere away from the box)
FREE_SPACE_POSE = deepcopy(positions["block_0"])
FREE_SPACE_POSE.pose.position.x -= 0.3 #check
FREE_SPACE_POSE.pose.position.z += 0.15 #check


def move_slow(mgc):
    mgc.set_max_velocity_scaling_factor(0.03)
    mgc.set_max_acceleration_scaling_factor(0.1)

def move_fast(mgc):
    mgc.set_max_velocity_scaling_factor(0.2)
    mgc.set_max_acceleration_scaling_factor(0.2)

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

'''
IN PROGRESS
check in lab for failure conditions
'''
def pick_up_cube_release(key): #fail helper
    #like pick_up_cube, but opens instead of closing: block dropped in place
    open_gripper()
    move_fast(mgc)
    if not move_to_pose(positions[key]):
        return False

    grasp = deepcopy(positions[key])
    grasp.pose.position.z -= DESCEND
    move_slow(mgc)
    if not move_to_pose(grasp):
        return False

    open_gripper()          #"drop" the block at the grasp position

    move_fast(mgc)
    if not move_to_pose(positions[key]):
        return False

    return True


def deposit_at_box():  #fail helper
    #used by fail modes only; normal path has this inline
    if not move_to_box():
        return False
    descend = deepcopy(mgc.get_current_pose())
    descend.pose.position.z -= 0.1
    if not move_to_pose(descend):
        return False
    open_gripper()
    rospy.sleep(rospy.Duration(1.0))

    #go up, dont hit box
    retreat = deepcopy(mgc.get_current_pose())
    retreat.pose.position.z += RETREAT_Z
    move_fast(mgc)
    if not move_to_pose(retreat):
        return False

    return True

def deposit_at(pose):  #fail helper, only for failed deposit area
    if not move_to_pose(pose):
        return False
    descend = deepcopy(mgc.get_current_pose())
    descend.pose.position.z -= 0.1
    if not move_to_pose(descend):
                    return False
    open_gripper()
    rospy.sleep(rospy.Duration(1.0))

    retreat = deepcopy(pose)
    retreat.pose.position.z += RETREAT_Z
    move_fast(mgc)
    if not move_to_pose(retreat):
        return False

    return True

# going to position of block and executing pick_up_cube()
def get_block_in_line(key, mode):
    # fail modes
    if mode == "repeated_drop": #failure condition 1
        rospy.loginfo("block %s: repeated_drop fail condition" % key)
        pick_up_cube_release(key)
        deposit_at_box()
        return False

    if mode == "empty_pickup": #failure condition 2
        rospy.loginfo("block %s: empty_pickup fail condition" % key)
        pick_up_cube("empty")
        deposit_at_box()
        return False

    if mode == "drop_in_space": #failure condition 3
        rospy.loginfo("block %s: drop_in_space fail condition" % key)
        if not pick_up_cube(key):
            return False
        deposit_at(FREE_SPACE_POSE) #set to failed deposit
        return False

    if not pick_up_cube(key): #regular
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
    todo = [k for k in positions.keys() if k != "empty"] #avoid failure conditions
    random.shuffle(todo)
else:
    todo = [k for k in positions.keys() if k != "empty"]

print("pick order: ", todo)

# fail assignment, capped and random
fail_assignment = {}
fails = 0
for key in todo:
    if fails < FAIL_COUNT and random.random() < FAIL_RATE:
        fail_assignment[key] = random.choice(FAIL_MODES)
        fails += 1
    else:
        fail_assignment[key] = None

print("fail assignment: ", fail_assignment)

#actual pick up and drop into box routine
for key in todo:
    ready()
    rospy.sleep(rospy.Duration(1.0))
    get_block_in_line(key, fail_assignment[key])
    rospy.sleep(rospy.Duration(1.0))
ready()

'''
#for testing:
key = "block_0" #or for key in todo:
mode = "drop_in_space"   # "repeated_drop" / "empty_pickup" / "drop_in_space"
ready()
rospy.sleep(rospy.Duration(1.0))
ok = get_block_in_line(key, mode)
rospy.loginfo("block %s mode=%s ok=%s" % (key, mode, ok))
ready()
'''