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
'''
IN PROGRESS check in lab
pose_z - DESCEND >= tabletop_z , don't go below the table top. (-0.205 >=-0.23
poze_z - DESCEND <= tabletop_z + h, you're at/below the block top, so you're actually on the block, not above it.
"random" failing
'''
DESCEND   = 0.20   # approach to grasp descent, tuned so gripper closes on block
RETREAT_Z = 0.20   # post-drop lift to clear box
RANDOMIZE = False   #True for actual, False for debugging
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
box_posture.position = [-0.110380859375, 0.35265234375, -0.2605810546875, -0.5528466796875, 0.25176171875, 1.7379130859375, 0.0980244140625]


#positions dict and keys
'''
IN PROGRESS
update with all 0-6 initial block positions in lab
make sure it is positioned a bit above the block, so it can crane game descend
'''

#hardcoded in lab
gap = 0.06      # y step between blocks in the same row, 6 cm apart center to center
GRASP_QUAT = (qx, qy, qz, qw)   #pls check, find from top grasp

anchor_even = geometry_msgs.msg.PoseStamped()
anchor_even.header.frame_id = "base"
anchor_even.pose.position.x = 0.52        #check pls, back row
anchor_even.pose.position.y = -0.06       #even rows start coordinate
anchor_even.pose.position.z = 0      #clearance above block

anchor_even.pose.orientation.x = GRASP_QUAT[0]
anchor_even.pose.orientation.y = GRASP_QUAT[1]
anchor_even.pose.orientation.z = GRASP_QUAT[2]
anchor_even.pose.orientation.w = GRASP_QUAT[3]

anchor_odd = deepcopy(anchor_even)
anchor_odd.pose.position.x = 0.58         #check pls, 6 cm ahead of back row
anchor_odd.pose.position.y = -0.08        #odd start, 2 cm off

positions = {}
for i in range(7):
    base = anchor_even if i % 2 == 0 else anchor_odd
    loc = deepcopy(base)
    loc.pose.position.y = base.pose.position.y + (i // 2) * gap
    positions["block_%d" % i] = loc

'''
Failure conditions 
'''
#for failure condition: empty pickup
anchor_empty = deepcopy(anchor_even)
anchor_empty.pose.position.x = 0.70          #check pls, should be clear of both rows
anchor_empty.pose.position.y = -0.06         #y coor with no block
positions["empty"] = anchor_empty

# free-space drop pose (check pls, anywhere away from the box)
FREE_SPACE_POSE = deepcopy(positions["block_0"])
FREE_SPACE_POSE.pose.position.x += 0.15 #check
FREE_SPACE_POSE.pose.position.z += 0.15 #check


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
    move_slow(mgc_global)
    if not move_to_box():
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
    move_slow(mgc)
    if not move_to_pose(pose):
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
        rospy.loginfo("block %s: repeated_drop failure" % key)
        pick_up_cube_release(key)
        deposit_at_box()
        return False

    if mode == "empty_pickup": #failure condition 2
        rospy.loginfo("block %s: empty_pickup failure" % key)
        pick_up_cube("empty")
        deposit_at_box()
        return False

    if mode == "drop_in_space": #failure condition 3
        rospy.loginfo("block %s: drop_in_space failure" % key)
        if not pick_up_cube(key):
            return False
        deposit_at(FREE_SPACE_POSE) #set to failed deposit
        return False

    if not pick_up_cube(key): #regular
        return False

    #move to box, and drop
    move_slow(mgc_global)
    if not move_to_box():
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
mode = "repeated_drop"  # "empty_pickup" / "drop_in_space"
ready()
rospy.sleep(rospy.Duration(1.0))
ok = get_block_in_line(key, mode)
rospy.loginfo("block %s mode=%s ok=%s" % (key, mode, ok))
ready()
'''