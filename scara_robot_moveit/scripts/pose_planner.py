#!/usr/bin/env python
# coding: UTF-8
import sys
from math import pi
import geometry_msgs.msg
import moveit_commander
import rospy
import tf
from geometry_msgs.msg import Quaternion, Vector3

def main():
    # moveit_commandern
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("pose_planner")
    
    # MoveGroupCommandern
    move_group = moveit_commander.MoveGroupCommander("scara_robot")
    print(move_group.get_current_pose().pose)
    
    pose_goal = geometry_msgs.msg.Pose()
    #pose_goal.position = Vector3(0.159922, 0.0, 0.11252700000000002)
    pose_goal.position = Vector3(0.1, 0.0, 0.11252700000000002)
    #q = tf.transformations.quaternion_from_euler(pi, 0, 0)
    q = tf.transformations.quaternion_from_euler(0, 0, 0)
    pose_goal.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
    move_group.set_pose_target(pose_goal)
    #move_group.set_joint_value_target(pose_goal, True)
    print(pose_goal)
    #move_group.go(wait=True)
    move_group.go()
    move_group.stop()
    move_group.clear_pose_targets()

if __name__ == "__main__":
    main()
