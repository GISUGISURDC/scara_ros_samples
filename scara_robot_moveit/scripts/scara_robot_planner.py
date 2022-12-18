#!/usr/bin/env python3
# coding: UTF-8
import sys
from math import pi
import moveit_commander
import rospy
def main():
    # moveit_commandern
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("scara_robot_planner")
    # MoveGroupCommandern
    robot = moveit_commander.RobotCommander()
    arm = moveit_commander.MoveGroupCommander("scara_robot")
    gripper = moveit_commander.MoveGroupCommander("gripper")

    print("Group names:")
    print(robot.get_group_names())

    print("Current state:")
    print(robot.get_current_state())

    arm_init_pose = arm.get_current_pose().pose
    print("Arm initial pose:")
    print(arm_init_pose)

    gripper_init_pose = gripper.get_current_pose().pose
    print("Gripper initial pose:")
    print(gripper_init_pose)
    gripper.set_joint_value_target([-0.005])
    gripper.go(wait=True)

    arm.set_named_target("home")
    arm.go(wait=True)
    
    joint_goal = [0, 0, -0.02, 0]
    arm.set_joint_value_target(joint_goal)
    arm.go(wait=True)
    
    gripper.set_joint_value_target([0.005])
    gripper.go(wait=True)

    joint_goal = [0, 0, 0.02, 0]
    arm.set_joint_value_target(joint_goal)
    arm.go(wait=True)

    joint_goal = [-30*pi/180, -90*pi/180, 0.02, 0]
    arm.set_joint_value_target(joint_goal)
    arm.go(wait=True)
        
    joint_goal = [-30*pi/180, -90*pi/180, -0.02, 0]
    arm.set_joint_value_target(joint_goal)
    arm.go(wait=True)

    gripper.set_joint_value_target([-0.005])
    gripper.go(wait=True)

    joint_goal = [-30*pi/180, -90*pi/180, -0.02, 0]
    arm.set_joint_value_target(joint_goal)
    arm.go(wait=True)

    joint_goal = [-30*pi/180, -90*pi/180, 0.02, 0]
    arm.set_joint_value_target(joint_goal)
    arm.go(wait=True)

    arm.set_named_target("home")
    arm.go(wait=True)

    arm.stop()

if __name__ == "__main__":
    main()
