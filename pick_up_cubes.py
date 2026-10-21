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

'''
IN PROGRESS 

create reference pose for box to drop cubes into

loc = geometry_msgs.msg.PoseStamped()
loc.header.frame_id = "base"
loc.pose.position.x = 0.8867910107919338
loc.pose.position.y = 0.07648888673307797
loc.pose.position.z = -0.15758351808118645

loc.pose.orientation.x = -0.021546827744775888
loc.pose.orientation.y = 0.9996207910915778
loc.pose.orientation.z = -0.010990924363512537
loc.pose.orientation.w = 0.013160843940309808
'''
#drop into box pose
box_posture = JointState()
box_posture.name = ["right_j0", "right_j1", "right_j2", "right_j3", "right_j4", "right_j5", "right_j6"]
box_posture.position = [-0.110380859375, 0.35265234375, -0.2605810546875, -0.5528466796875, 0.25176171875, 1.7379130859375, 0.0980244140625]


#positions dict and keys
positions = []
'''
IN PROGRESS

update with all 6-9 initial block positions
make sure it is positioned a bit above the block, so it can crane game descend
'''

#position 1 
'''
pose: 
  position: 
    x: 0.5214111091560647
    y: 0.0950580525914052
    z: -0.07324304088056395
  orientation: 
    x: -0.04405131640240166
    y: 0.9989142756984348
    z: -0.015098336608216226
    w: 0.0013384919695063212 ]
'''
#index= "middle" REWRITE: joint limited
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

right side, 1st from middle
pose: 
  position: 
    x: 0.5134768375525847
    y: 0.19486823461994582
    z: -0.08840145571286147
  orientation: 
    x: -0.05957733802638406
    y: 0.9973270844216934
    z: -0.0357128136858031
    w: 0.022667651207350877 ]

'''


'''
left side, 1st from middle'
pose: 
  position: 
    x: 0.538209189806123
    y: -0.026163267084268308
    z: -0.08983185293090185
  orientation: 
    x: 0.057846047522912114
    y: -0.9977937499870082
    z: 0.03229415482577637
    w: 0.00430753258026647 ]

'''



def move_slow(mgc):
    mgc.set_max_velocity_scaling_factor(0.03)
    mgc.set_max_acceleration_scaling_factor(0.1)

def move_fast(mgc):
    mgc.set_max_velocity_scaling_factor(1.0)
    mgc.set_max_acceleration_scaling_factor(1.0)

def move_to_box():
    mgc_global.set_joint_value_target(box_posture)
    mgc_global.go(wait=True)

def move_to_pose(pose):
    mgc.set_pose_target(pose)
    mgc.go(wait=True)

def ready():
    mgc_global.set_named_target("ready")
    mgc_global.go(wait=True)

def pick_up_cube(index):
    open_gripper()
    move_fast(mgc)
    move_to_pose(positions[index])

    #lower
    grasp = deepcopy(positions[index])
    grasp.pose.position.z -= 0.15
    move_slow(mgc)
    move_to_pose(grasp)

    #close
    close_gripper()

    #go up
    move_fast(mgc)
    move_to_pose(positions[index])

'''
IN PROGRESS
'''
def get_block_in_line(index):
    pick_up_cube(index)

    #move to box, and drop IN PROGRESS
    move_slow(mgc)
    move_to_box()
    open_gripper()
    rospy.sleep(rospy.Duration(1.0))

    #move up so it doesnt hit block (NEEDS FIXING)
    retreat = deepcopy(mgc.get_current_pose())
    retreat.pose.position.z += 0.20
    move_fast(mgc)
    move_to_pose(retreat)

#say("Hello, let's try to drop some blocks.")
for i in range(len(positions)):
    ready()
    rospy.sleep(rospy.Duration(1.0))
    get_block_in_line(i)
    rospy.sleep(rospy.Duration(1.0))
ready()


#IN PROGRESS, random selector of initial positions
'''
import random

todo = list(positions.keys())
random.shuffle(todo)

for i in todo:
    ready()
    rospy.sleep(rospy.Duration(1.0))
    get_block_in_line(i)
    rospy.sleep(rospy.Duration(1.0))
ready()
'''

#say("All done.")


'''
ERROR
[INFO] [1792595505.938158200]: Ready to take commands for planning group arm.
[INFO] [1792595506.330321975]: Ready to take commands for planning group arm_local.
[INFO] [1792595519.746130337]: ABORTED: UNKNOWN
Traceback (most recent call last):
  File "/home/hri26-group5/ros/src/whiny_bot/pick_up_cubes.py", line 222, in <module>
    get_block_in_line(i)
  File "/home/hri26-group5/ros/src/whiny_bot/pick_up_cubes.py", line 201, in get_block_in_line
    move_to_pose(box_posture)
  File "/home/hri26-group5/ros/src/whiny_bot/pick_up_cubes.py", line 168, in move_to_pose
    mgc.set_pose_target(pose)
  File "/opt/ros/one/lib/python3/dist-packages/moveit_commander/move_group.py", line 345, in set_pose_target
    ok = self._g.set_pose_target(pose, end_effector_link)
         ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Boost.Python.ArgumentError: Python argument types in
    MoveGroupInterface.set_pose_target(MoveGroupInterface, JointState, str)
did not match C++ signature:
    set_pose_target(moveit::planning_interface::MoveGroupInterfaceWrapper {lvalue}, boost::python::list {lvalue}, std::__cxx11::basic_string<char, std::char_traits<char>, std::allocator<char> >)

'''