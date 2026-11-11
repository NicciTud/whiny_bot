#!/usr/bin/env python
'''
Has failure conditions, for testing

'''
import sys
import rospy
import rospkg
import moveit_commander
import geometry_msgs.msg
from copy import deepcopy
from control_msgs.msg import GripperCommand
from sensor_msgs.msg import JointState
from std_msgs.msg import String as StringMsg, Bool, Float32
import os, random
from sound_play.libsoundplay import SoundClient
import intera_interface

from sensor_msgs.msg import Image


DESCEND   = 0.18   # approach to grasp descent, tuned so gripper closes on block
RETREAT_Z = 0.30   # post-drop lift to clear box
RANDOMIZE = True   #True for actual, False for debugging
FAIL_COUNT = 2          #how many MAX blocks fail per run
FAIL_RATE  = 0.5        #probability a block fails *given* it's chosen
FAIL_MODES = ["repeated_drop", "empty_pickup", "drop_in_space"]


#gripper commands, open close
def main():
    rospy.init_node('gestures_pickup', anonymous=False)
    #Head Movement Publisher
    #Enable is to set its state False = Default position[0 degrees, True = Head move
    enable_pub = rospy.Publisher("looking_at_person/enable", Bool, queue_size=1, latch=True)
    #Sets the angle the head needs to move
    angle_pub  = rospy.Publisher("looking_at_person/panning_angle", Float32, queue_size=1)
    #Sets the speed
    speed_pub = rospy.Publisher("looking_at_person/speed", Float32, queue_size=1)

    #Enables Sounds to be played
    sound = SoundClient()
    rospy.sleep(1)
    #sets the image publisher
    head_display = intera_interface.HeadDisplay()
    pkg_path = rospkg.RosPack().get_path('whiny_bot')
    head_display.display_image(f"{pkg_path}/faces/neutral.png")

        # bail out if Ctrl+C was hit
    def shutdown():
        return rospy.is_shutdown()
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
    FREE_SPACE_POSE.pose.position.x += 0.3 #check
    FREE_SPACE_POSE.pose.position.z += 0.2 #check
    '''
    Screen Gestures - TO-DO:
    Shake head (>.<) in frustration
    spin 360 in joy (spams happy noises,Maybe have it raise its arm high)
    side eyes when it drops a block (Whoops kinda sound)
    Focused look when its trying to grab the blocks 
    # person detected
    angle_pub.publish(0.5)        # angle in radians (or Float32(0.5), both work)
    enable_pub.publish(True)

    # person gone
    enable_pub.publish(False)
    '''

    EXPRESSIONS = {
        "jubilation": {
            "faces":  ["jubilation.png"],
            "sounds": ["jubilation.wav", "jubilation2.wav", "jubilation3.wav"],
        },
        "stress": {
            "faces":  ["stressed.png"],
            "sounds": ["stressed.wav", "stressed2.wav"],
        },
        "sideeye": {
            "faces":  ["sideeye.png"],
            "sounds": ["sideeye.wav", "whoops.wav"],
        },
        "focused": {
            "faces":  ["focused.png"],
            "sounds": ["focused.wav","focused1.wav","focused3.wav"],
        },
    }

    _last = {}

    def pick(name, kind):
        options = EXPRESSIONS[name][kind]
        last = _last.get((name, kind))

        choices = []
        for o in options:
            if o != last:
                choices.append(o)

        if not choices:
            choices = options

        choice = random.choice(choices)
        _last[(name, kind)] = choice
        return choice

    def play_expression(name):
        head_display.display_image(f"{pkg_path}/faces/{pick(name, 'faces')}")
        sound.playWave(f"{pkg_path}/sfx/{pick(name, 'sounds')}")
    
    
    def jubilation():

        play_expression("jubilation")
        speed_pub.publish(0.6)
        angle_pub.publish(-0.3)
        enable_pub.publish(True)
        rospy.sleep(0.4)
        for angle in (0.3, -0.3, 0.3, -0.3, 0.3):
            #i put hand motion here#
            angle_pub.publish(angle)
            rospy.sleep(0.4)
        enable_pub.publish(False)
            #TO-DO: ADD arm raise gesture

    def stress():
        play_expression("stress")
        speed_pub.publish(0.9)
        angle_pub.publish(-0.4)
        enable_pub.publish(True)
        rospy.sleep(0.5)

        #jittery head shake delay is random 
        for angle in (0.5, -0.2, 0.35, -0.4, 0.1, -0.3, 0.6):
            angle_pub.publish(angle)
            rospy.sleep(random.uniform(0.2, 0.5))

        # goes back to normall pos and trembles a bit
        angle_pub.publish(0.0)
        rospy.sleep(0.6)
        for i in range(8):
            angle_pub.publish(random.uniform(-0.08, 0.08))
            rospy.sleep(0.12)
        #calming down 
        speed_pub.publish(0.3) 
        enable_pub.publish(False)

    def side_eye():
        play_expression("sideeye")
        speed_pub.publish(0.2)
        angle_pub.publish(-0.4)
        enable_pub.publish(True)
        rospy.sleep(0.4)
        speed_pub.publish(0.7) 
        enable_pub.publish(False)


    def focused_toblock():
        play_expression("focused")
        speed_pub.publish(0.3) 
        angle_pub.publish(-0.2)    
        enable_pub.publish(True)

    def focused_tobox():
        play_expression("focused")
        speed_pub.publish(0.3)
        enable_pub.publish(False)   
        head_display.display_image(f"{pkg_path}/faces/neutral.png")


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
        focused_toblock()
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
        focused_tobox()
        return True

    '''
    IN PROGRESS
    check in lab for failure conditions
    '''
    def pick_up_cube_release(key): #fail helper
        #like pick_up_cube, but opens instead of closing: block dropped in place
        open_gripper()
        focused_toblock()
        move_fast(mgc)
        if not move_to_pose(positions[key]):
            return False

        grasp = deepcopy(positions[key])
        grasp.pose.position.z -= DESCEND
        move_slow(mgc)
        if not move_to_pose(grasp):
            return False

        open_gripper()          #"drop" the block at the grasp position
        side_eye()
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
        side_eye()
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
            stress()
            return False

        if mode == "empty_pickup": #failure condition 2
            rospy.loginfo("block %s: empty_pickup fail condition" % key)
            pick_up_cube("empty")
            deposit_at_box()
            stress()
            return False

        if mode == "drop_in_space": #failure condition 3
            rospy.loginfo("block %s: drop_in_space fail condition" % key)
            if not pick_up_cube(key):
                return False
            deposit_at(FREE_SPACE_POSE) #set to failed deposit
            stress()
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
        jubilation()
        return True

#actual start
    if shutdown():
        return
    key = "block_0"
    mode = ""
    ready()
    rospy.sleep(rospy.Duration(1.0))
    ok = get_block_in_line(key, mode)
    rospy.loginfo("block %s mode=%s ok=%s" % (key, mode, ok))
    ready()

    if shutdown():
        return
    key = "block_3"
    mode = "drop_in_space"
    ready()
    rospy.sleep(rospy.Duration(1.0))
    ok = get_block_in_line(key, mode)
    rospy.loginfo("block %s mode=%s ok=%s" % (key, mode, ok))
    ready()

    if shutdown():
        return
    key = "block_5"
    mode = ""
    ready()
    rospy.sleep(rospy.Duration(1.0))
    ok = get_block_in_line(key, mode)
    rospy.loginfo("block %s mode=%s ok=%s" % (key, mode, ok))
    ready()

    if shutdown():
        return
    key = "block_1"
    mode = "empty_pickup"
    ready()
    rospy.sleep(rospy.Duration(1.0))
    ok = get_block_in_line(key, mode)
    rospy.loginfo("block %s mode=%s ok=%s" % (key, mode, ok))
    ready()

    if shutdown():
        return
    key = "block_4"
    mode = ""
    ready()
    rospy.sleep(rospy.Duration(1.0))
    ok = get_block_in_line(key, mode)
    rospy.loginfo("block %s mode=%s ok=%s" % (key, mode, ok))
    ready()

    if shutdown():
        return
    key = "block_2"
    mode = ""
    ready()
    rospy.sleep(rospy.Duration(1.0))
    ok = get_block_in_line(key, mode)
    rospy.loginfo("block %s mode=%s ok=%s" % (key, mode, ok))
    ready()

    if shutdown():
        return
    key = "block_6"
    mode = ""
    ready()
    rospy.sleep(rospy.Duration(1.0))
    ok = get_block_in_line(key, mode)
    rospy.loginfo("block %s mode=%s ok=%s" % (key, mode, ok))
    ready()

if __name__ == '__main__':
    main()