#!/usr/bin/env python

import rospy
import copy
import moveit_commander
import moveit_msgs.msg
from geometry_msgs.msg import Pose
from moveit_commander.conversions import pose_to_list
import time
import math
import tf

def getCartPath(group, trans, rot):
    waypoints = []

    tool_pose = Pose()
    tool_pose.position.x = trans[0]
    tool_pose.position.y = trans[1]
    tool_pose.position.z = trans[2]
    tool_pose.orientation.x = rot[0]
    tool_pose.orientation.y = rot[1]
    tool_pose.orientation.z = rot[2]
    tool_pose.orientation.w = rot[3]

    waypoints.append(copy.deepcopy(tool_pose))

    (plan, fraction) = group.compute_cartesian_path(waypoints, 0.002, 0.0)

    return plan, fraction

def main():
    rospy.init_node('arm_controller', anonymous=True)
    robot = moveit_commander.RobotCommander()
    group = moveit_commander.MoveGroupCommander("manipulator")

    listener = tf.TransformListener()

    group.set_max_velocity_scaling_factor(0.5)
    group.set_max_acceleration_scaling_factor(0.5)

    group.go([0.5, 0, 0, -1.5, 0, 1.5, 0.5], wait=True)
    group.stop()

    r = rospy.Rate(1)
    while not rospy.is_shutdown():
        listener.waitForTransform('world', 'operation_tool', rospy.Time(0), rospy.Duration(0.5))
        trans, rot = listener.lookupTransform('world', 'operation_tool', rospy.Time(0))

        plan, fraction = getCartPath(group, trans, rot)
        if fraction > 0.95:
            group.execute(plan, wait=True)
        else:
            rospy.logwarn("Incomplete path planning")
        r.sleep()

    group.stop()

if __name__ == '__main__':
    main()
