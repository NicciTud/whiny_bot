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
say_pub = rospy.Publisher('/say', StringMsg, queue_size=1)
def say(text):
    msg = StringMsg()
    msg.data = text
    say_pub.publish(msg)

def wait_for_external_effort(min_effort=3.0):
    recent_efforts = []
    import threading
    condition = threading.Condition()
    def callback(data):
        nonlocal sub
        # detect and signal when effort in some joint differs above threshold from median over last second
        recent_efforts.append((data.header.stamp, data.effort))
        # keep only last second of efforts
        cutoff_time = rospy.Time.now() - rospy.Duration(1.0)
        while recent_efforts and recent_efforts[0][0] < cutoff_time:
            recent_efforts.pop(0)
        if len(recent_efforts) < 25:
            return
        # compute median effort for each joint
        import numpy as np
        efforts_array = np.array([e for t, e in recent_efforts])
        median_efforts = np.median(efforts_array, axis=0)
        # check if any joint effort differs from median by more than min_effort
        for effort in data.effort:
            if abs(effort - median_efforts[data.effort.index(effort)]) > min_effort:
                # signal condition
                with condition:
                    say("beep")
                    sub.unregister()
                    condition.notify_all()
                return
    
    sub = rospy.Subscriber('/robot/joint_states', JointState, callback)
    # wait for signal, timeout after 8 seconds
    with condition:
        condition.wait(timeout=8.0)
    sub.unregister()

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
table_pose.pose.position.y = -0.1
table_pose.pose.position.z = -0.28
table_pose.pose.orientation.w = 1.0
psi.add_box("table", pose=table_pose, size=(0.8, 1.1, 0.2))

window_collider_pose = geometry_msgs.msg.PoseStamped()
window_collider_pose.header.frame_id = "base"
window_collider_pose.pose.position.x = 0.3
window_collider_pose.pose.position.y = 0.6
window_collider_pose.pose.position.z = 0.0
window_collider_pose.pose.orientation.w = 1.0
psi.add_box("window_collider", pose=window_collider_pose, size=(1.6, 0.2, 1.5))

handover_posture = JointState()
handover_posture.name = ["right_j0", "right_j1", "right_j2", "right_j3", "right_j4", "right_j5", "right_j6"]
handover_posture.position = [-0.24144140625, 1.0161748046875, -0.1290107421875, -2.0135302734375, -0.1491611328125, 1.2109267578125, -0.000138671875]


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

def move_to_handover():
    mgc_global.set_joint_value_target(handover_posture)
    mgc_global.go(wait=True)
def move_to_pose(pose):
    mgc.set_pose_target(pose)
    mgc.go(wait=True)
def ready():
    mgc_global.set_named_target("ready")
    mgc_global.go(wait=True)

def place_cube(color):
    if color not in cubes:
        raise ValueError(f"No pose defined for color '{color}'")
    move_to_handover()
    say(f"Please hand me the {color} cube.")
    wait_for_external_effort()
    close_gripper()
    t = deepcopy(cubes[color])
    t.pose.position.z += 0.05
    move_fast(mgc)
    move_to_pose(t)
    move_slow(mgc)
    move_to_pose(cubes[color])
    open_gripper()
    move_fast(mgc)
    move_to_pose(t)

open_gripper()
ready()
say("Alright, we have to arrange all these blocks properly. Please pass them to me one by one.")
rospy.sleep(rospy.Duration(5.0))

import random
todo = list(cubes.keys())
random.shuffle(todo)
for color in todo:
    place_cube(color)
ready()
say("All done, thank you for helping me out!")