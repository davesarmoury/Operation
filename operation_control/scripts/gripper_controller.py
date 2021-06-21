#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from robotiq_2f_gripper_control.msg import Robotiq2FGripper_robot_output

OPEN = 128
CLOSE = 255
SPEED = 10
FORCE = 5

def halfOpenMsg():
    msg = openMsg()
    msg.rPR = int((OPEN + CLOSE)/2)
    return msg

def closeMsg():
    msg = openMsg()
    msg.rPR = CLOSE
    return msg

def openMsg():
    msg = activateMsg()
    msg.rPR = OPEN
    msg.rSP = SPEED
    msg.rFR = FORCE
    return msg

def activateMsg():
    msg = Robotiq2FGripper_robot_output()
    msg.rACT = 1
    msg.rGTO = 1
    msg.rATR = 0
    msg.rPR = 0
    msg.rSP = 255
    msg.rFR = 150
    return msg

def resetMsg():
    msg = Robotiq2FGripper_robot_output()
    msg.rACT = 0
    msg.rGTO = 0
    msg.rATR = 0
    msg.rPR = 0
    msg.rSP = 0
    msg.rFR = 0
    return msg

def update_gripper_callback(msg):
    global pub
    if msg.data.upper() == "RET":
        pub.publish(resetMsg())
        pub.publish(halfOpenMsg())
    if msg.data.upper() == "GO":
        pub.publish(resetMsg())
        pub.publish(openMsg())
    if msg.data.upper() == "GC":
        pub.publish(resetMsg())
        pub.publish(closeMsg())
    if msg.data.upper() == "GH":
        pub.publish(resetMsg())
        pub.publish(halfOpenMsg())

if __name__ == '__main__':
    global pub
    rospy.init_node('operation_gripper_controller')

    rospy.Subscriber("vote_result", String, update_gripper_callback)
    pub = rospy.Publisher('/Robotiq2FGripperRobotOutput', Robotiq2FGripper_robot_output, queue_size=10)

    rospy.sleep(3.0)
    pub.publish(resetMsg())
    rospy.sleep(1.0)
    pub.publish(activateMsg())
    rospy.sleep(2.0)

    rospy.spin()
