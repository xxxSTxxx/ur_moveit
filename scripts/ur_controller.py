# import rospy
# import math
# import numpy as np

# from std_msgs.msg import Float64
# from math import sin,cos,atan2,sqrt,fabs,pi

# # Define a robot joint positions publisher for joint controllers.
# def joint_positions_publisher():

#     # Initiate node for controlling joint1 and joint2 positions.
#     rospy.init_node('ros_control_position_publisher', anonymous=False)

#     pub = rospy.Publisher('/arm_controller/command', Float64, queue_size=10)

#     rate = rospy.Rate(0.1) #100 Hz

#     # drag robot to initial state
#     state_ini = [0, 0, 0, 0, 0, 0]
#     publish2joint(pub, state_ini)

#     rate.sleep()


# def publish2joint(pub, signals):
#     pub.publish(signals)


# # Main section of code that will continuously run unless rospy receives interuption (ie CTRL+C)
# if __name__ == '__main__':
#     joint_positions_publisher()

#! /usr/bin/env python
import sys
import rospy
import moveit_commander
import geometry_msgs

moveit_commander.roscpp_initializer.roscpp_initialize(sys.argv)
rospy.init_node('move_group_grasp', anonymous=True)
robot = moveit_commander.robot.RobotCommander()

arm_group = moveit_commander.move_group.MoveGroupCommander("arm")
# hand_group = moveit_commander.move_group.MoveGroupCommander("gripper")
arm_group.set_named_target("home_j")
plan = arm_group.go()
# hand_group.set_named_target("open")
# plan = hand_group.go()

pose_target = geometry_msgs.msg.Pose()
pose_target.orientation.w = 0.5
pose_target.orientation.x = -0.5
pose_target.orientation.y = 0.5
pose_target.orientation.z = -0.5
pose_target.position.x = 0.15
pose_target.position.y = 0
pose_target.position.z = 1.25
arm_group.set_pose_target(pose_target)
plan = arm_group.go()

pose_target.position.z = 1.08
arm_group.set_pose_target(pose_target)
plan = arm_group.go()

# hand_group.set_named_target("close")
# plan = hand_group.go()

pose_target.position.z = 1.5
arm_group.set_pose_target(pose_target)
plan = arm_group.go()

# hand_group.set_named_target("open")
# plan = hand_group.go()

rospy.sleep(5)
moveit_commander.roscpp_initializer.roscpp_shutdown()