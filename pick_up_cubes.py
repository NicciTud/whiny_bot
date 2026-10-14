#!/usr/bin/env python

import sys
import rospy
import moveit_commander
import geometry_msgs.msg
from copy import deepcopy
from control_msgs.msg import GripperCommand
from sensor_msgs.msg import JointState
from std_msgs.msg import String as StringMsg

rospy.init_node('cube_arrangement_demo', anonymous=False)

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

table_pose = geometry_msgs.msg.PoseStamped()
table_pose.header.frame_id = "base"
table_pose.pose.position.x = 0.75
table_pose.pose.position.y = -0.13
table_pose.pose.position.z = -0.28
table_pose.pose.orientation.w = 1.0
psi.add_box("table", pose=table_pose, size=(0.8, 0.8, 0.10)) #edited table size

window_collider_pose = geometry_msgs.msg.PoseStamped()
window_collider_pose.header.frame_id = "base"
window_collider_pose.pose.position.x = 0.3
window_collider_pose.pose.position.y = 0.6
window_collider_pose.pose.position.z = 0.0
window_collider_pose.pose.orientation.w = 1.0
psi.add_box("window_collider", pose=window_collider_pose, size=(1.6, 0.2, 1.5))


#waiting to pickup blocks pose, replace with move to top grasp
pickup_posture = JointState()
pickup_posture.name = ["right_j0", "right_j1", "right_j2", "right_j3", "right_j4", "right_j5", "right_j6"]
pickup_posture.position = [-0.051150390625, -0.831271484375, -0.1108291015625, 2.091619140625, 0.1726298828125, 0.3259853515625, -0.2090615234375]

'''
IN PROGRESS 

REFERENCE: green-yellow

can be used to build tower borders, change poses and positions
blue-orange needs to be more forward, block pickup location forward
save poses

'''
#array of pos
positions = []

#position 1
loc = geometry_msgs.msg.PoseStamped()
loc.header.frame_id = "base"
loc.pose.position.x = 0.8867910107919338
loc.pose.position.y = 0.07648888673307797
loc.pose.position.z = -0.15758351808118645

loc.pose.orientation.x = -0.021546827744775888
loc.pose.orientation.y = 0.9996207910915778
loc.pose.orientation.z = -0.010990924363512537
loc.pose.orientation.w = 0.013160843940309808

positions.append(loc)
'''

cubes= dict()

color="red" #fix, too far
cube = geometry_msgs.msg.PoseStamped()
cube.header.frame_id = "base"
cube.pose.position.x = 0.697
cube.pose.position.y = -0.369
cube.pose.position.z = -0.152
cube.pose.orientation.x = -0.188
cube.pose.orientation.y = 0.981
cube.pose.orientation.z = -0.020
cube.pose.orientation.w = 0.035
cubes[color] = cube

color = "green"
cube = geometry_msgs.msg.PoseStamped()
cube.header.frame_id = "base"
cube.pose.position.x = 0.876844
cube.pose.position.y = -0.0194747
cube.pose.position.z = -0.155714
cube.pose.orientation.x = -0.0274387
cube.pose.orientation.y = 0.996718
cube.pose.orientation.z = -0.0109193
cube.pose.orientation.w = 0.0753798
cubes[color] = cube

color = "yellow"
cube = geometry_msgs.msg.PoseStamped()
cube.header.frame_id = "base"
cube.pose.position.x = 0.787161
cube.pose.position.y = 0.28444
cube.pose.position.z = -0.149635
cube.pose.orientation.x = -0.292078
cube.pose.orientation.y = 0.955773
cube.pose.orientation.z = -0.0124869
cube.pose.orientation.w = 0.0321393
cubes[color] = cube
'''

def move_slow(mgc):
    mgc.set_max_velocity_scaling_factor(0.03)
    mgc.set_max_acceleration_scaling_factor(0.1)

def move_fast(mgc):
    mgc.set_max_velocity_scaling_factor(1.0)
    mgc.set_max_acceleration_scaling_factor(1.0)

def move_to_pickup():
    mgc_global.set_joint_value_target(pickup_posture)
    mgc_global.go(wait=True)

def move_to_pose(pose):
    mgc.set_pose_target(pose)
    mgc.go(wait=True)

def ready():
    mgc_global.set_named_target("ready")
    mgc_global.go(wait=True)

def pick_up_cube():
    '''
    IN PROGRESS
    #go lower
    pickup_pose = mgc.get_current_pose()
    pickup_pose.pose.position.z-=0.15
    
    mgc.set_pose_target(pickup_pose)
    mgc.go(wait=True)
    mgc.clear_pose_targets
''' 
    
    move_to_pickup()
    open_gripper()
    #go lower
    pickup_pose = mgc.get_current_pose()
    pickup_pose.pose.position.z-=0.15
    move_fast(mgc)
    move_to_pose(pickup_pose)
    close_gripper()
    pickup_pose.pose.position.z += 0.2
    move_to_pose(pickup_pose)

def stack_pyramid(index):
    t = deepcopy(positions[index])
    move_to_pose(t)
    t.pose.position.z -= 0.02
    move_to_pose(t)
    open_gripper()
    rospy.sleep(rospy.Duration(1.0))
    #move up so it doesnt hit block (NEEDS FIXING)
    #t.pose.position.z += 0.3
    #move_to_pose(t)

'''
def place_cube(color):
    if color not in cubes:
        raise ValueError(f"No pose defined for color '{color}'")
    move_to_pickup()
    say(f"Please hand me the {color} cube.")
    # wait_for_external_effort()
    close_gripper()



    t = deepcopy(cubes[color])
    t.pose.position.z += 0.10
    move_fast(mgc)
    move_to_pose(t)
    move_slow(mgc)
    move_to_pose(cubes[color])
    open_gripper()
    move_fast(mgc)
    move_to_pose(t)
'''

#say("Hello, let's try to build a tower.")
for i in range(len(positions)):
    ready()
    rospy.sleep(rospy.Duration(1.0))
    pick_up_cube()
    rospy.sleep(rospy.Duration(1.0))
    stack_pyramid(i)
ready()




'''
replace with normal pick up, place

import random
todo = list(cubes.keys())
random.shuffle(todo)
for color in todo:
    place_cube(color)
    ready()
'''

#say("All done.")