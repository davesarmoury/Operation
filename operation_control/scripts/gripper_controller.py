#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from robotiq_2f_gripper_control.msg import Robotiq2FGripper_robot_output

OPEN = 128
CLOSE = 255
SPEED = 10
FORCE = 5

def partOpenMsg(amount):
    msg = openMsg()
    msg.rPR = int(OPEN + (CLOSE - OPEN) * amount)
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
        pub.publish(partOpenMsg(0.5))
    if msg.data.upper() == "GO":
        pub.publish(resetMsg())
        pub.publish(openMsg())
    if msg.data.upper() == "GC":
        pub.publish(resetMsg())
        pub.publish(closeMsg())
    if msg.data.upper() == "G1":
        pub.publish(resetMsg())
        pub.publish(partOpenMsg(0.1))
    if msg.data.upper() == "G2":
        pub.publish(resetMsg())
        pub.publish(partOpenMsg(0.2))
    if msg.data.upper() == "G3":
        pub.publish(resetMsg())
        pub.publish(partOpenMsg(0.3))
    if msg.data.upper() == "G4":
        pub.publish(resetMsg())
        pub.publish(partOpenMsg(0.4))
    if msg.data.upper() == "G5":
        pub.publish(resetMsg())
        pub.publish(partOpenMsg(0.5))
    if msg.data.upper() == "G6":
        pub.publish(resetMsg())
        pub.publish(partOpenMsg(0.6))
    if msg.data.upper() == "G7":
        pub.publish(resetMsg())
        pub.publish(partOpenMsg(0.7))
    if msg.data.upper() == "G8":
        pub.publish(resetMsg())
        pub.publish(partOpenMsg(0.8))
    if msg.data.upper() == "G9":
        pub.publish(resetMsg())
        pub.publish(partOpenMsg(0.9))


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
