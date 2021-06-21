#!/usr/bin/env python

import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
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

    (plan, fraction) = group.compute_cartesian_path(waypoints, 0.005, 0.0)

    return plan, fraction

def main():
    rospy.init_node('arm_controller', anonymous=True)
    robot = moveit_commander.RobotCommander()
    group = moveit_commander.MoveGroupCommander("manipulator")

    listener = tf.TransformListener()

    # We can get the name of the reference frame for this robot:
    planning_frame = group.get_planning_frame()
    print "============ Reference frame: %s" % planning_frame

    # We can also print the name of the end-effector link for this group:
    eef_link = group.get_end_effector_link()
    print "============ End effector: %s" % eef_link

    # We can get a list of all the groups in the robot:
    group_names = robot.get_group_names()
    print "============ Robot Groups:", robot.get_group_names()

    # Sometimes for debugging it is useful to print the entire state of the
    # robot:
    print "============ Printing robot state"
    print robot.get_current_state()
    print ""

    group.set_max_velocity_scaling_factor(0.5)
    group.set_max_acceleration_scaling_factor(0.5)

    r = rospy.Rate(1)
    while not rospy.is_shutdown():
        waitForTransform('/operation_tool', '/world', rospy.Time(0), rospy.Duration(0.5))
        (trans, rot) = listener.lookupTransform('/operation_tool', '/world', rospy.Time(0))

        plan, fraction = getCartPath(group, trans, rot)
        if fraction > 0.95:
            group.execute(plan, wait=True)
        else:
            rospy.logwarn("Incomplete path planning")
        r.sleep()

    group.stop()

    if __name__ == '__main__':
        main()
