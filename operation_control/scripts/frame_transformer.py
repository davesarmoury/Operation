#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import TransformStamped
from tf.transformations import *
import tf
import numpy

X_MIN=-0.1
X_MAX=0.3
Y_MIN=0.0
Y_MAX=0.5
Z_MIN=0.0
Z_MAX=0.3
TILT_MIN=-1.0
TILT_MAX=1.0

MIN_SPEED = 0.002
MAX_SPEED = 0.1 - MIN_SPEED

def reset_frame():
    return compose_matrix(translate=[0.1,0.3,0.3])

def update_frame_callback(msg):
    global tool_mtx

    if msg.data.upper() == "HOME":
        tool_mtx = reset_frame()
        return

    scale, shear, angles, translate, perspective = decompose_matrix(tool_mtx)
    lin = ((translate[2] - Z_MIN)/Z_MAX) * MAX_SPEED + MIN_SPEED
    rot = lin
    rospy.loginfo(str(lin))

    if msg.data.upper() == "TX+":
        shift = compose_matrix(translate=(lin,0,0))
    elif msg.data.upper() == "TX-":
        shift = compose_matrix(translate=(-lin,0,0))
    elif msg.data.upper() == "TY+":
        shift = compose_matrix(translate=(0,lin,0))
    elif msg.data.upper() == "TY-":
        shift = compose_matrix(translate=(0,-lin,0))
    elif msg.data.upper() == "TZ+":
        shift = compose_matrix(translate=(0,0,lin))
    elif msg.data.upper() == "TZ-":
        shift = compose_matrix(translate=(0,0,-lin))
    elif msg.data.upper() == "RX+":
        shift = compose_matrix(angles=(rot,0,0))
    elif msg.data.upper() == "RX-":
        shift = compose_matrix(angles=(-rot,0,0))
    elif msg.data.upper() == "RY+":
        shift = compose_matrix(angles=(0,rot,0))
    elif msg.data.upper() == "RY-":
        shift = compose_matrix(angles=(0,-rot,0))
    elif msg.data.upper() == "RZ+":
        shift = compose_matrix(angles=(0,0,rot))
    elif msg.data.upper() == "RZ-":
        shift = compose_matrix(angles=(0,0,-rot))
    elif msg.data.upper() == "RET":
        shift = compose_matrix(translate=(0,0,0.1))
    else:
        rospy.logdebug("Not a frame command " + str(msg.data))
        return

    tmp_mtx = numpy.matmul(tool_mtx, shift)
    ####################
    scale, shear, angles, translate, perspective = decompose_matrix(tmp_mtx)
    translate_limit = [max(min(translate[0],X_MAX),X_MIN), max(min(translate[1],Y_MAX),Y_MIN), max(min(translate[2],Z_MAX),Z_MIN)]
    angles_limit = [max(min(angles[0],TILT_MAX),TILT_MIN), max(min(angles[1],TILT_MAX),TILT_MIN), angles[2]]
    ######################
    tool_mtx = compose_matrix(translate=translate_limit, angles=angles_limit)

if __name__ == '__main__':
    global tool_mtx
    tool_mtx = reset_frame()
    rospy.init_node('operation_frame_broadcaster')
    br = tf.TransformBroadcaster()
    tool_mtx = reset_frame()

    rospy.Subscriber("vote_result", String, update_frame_callback)

    r = rospy.Rate(5) # 5hz
    while not rospy.is_shutdown():
        scale, shear, angles, translate, perspective = decompose_matrix(tool_mtx)
        br.sendTransform(translate,
                 quaternion_from_euler(angles[0], angles[1], angles[2]),
                 rospy.Time.now(),
                 "operation_tool",
                 "world")

        r.sleep()
