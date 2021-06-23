#!/usr/bin/env python
import rospy
import operator
import copy
from std_msgs.msg import String
from twitch_chat_ros.msg import twitch_message

vote_seconds = 6
VALID_COMMANDS = ["GO","GC","GH","TX+","TX-","TY+","TY-","TZ+","TZ-","RX+","RX-","RY+","RY-","RZ+","RZ-"]
register = {}

def callback(msg):
    global register
    if len(msg.arguments) == 1:
        if msg.arguments[0] in VALID_COMMANDS:
            register[msg.tags.display_name] = msg.arguments[0]

def main():
    global register
    pub = rospy.Publisher('vote_result', String, queue_size=10)
    pub2 = rospy.Publisher('send_msg', String, queue_size=10)
    rospy.Subscriber('received_message', twitch_message, callback)

    rospy.init_node('vote_coordinator', anonymous=True)

    rate = rospy.Rate(10)
    while rospy.get_time() <= 0.1:
        pass

    next_election = rospy.Time.now() + rospy.Duration(vote_seconds)

    while not rospy.is_shutdown():
        if rospy.Time.now() >= next_election:
            rospy.logdebug(str(register))
            if len(register) > 0:
                temp_register = copy.deepcopy(register)
                register = {}
                vote_tally = {}
                for vote in temp_register:
                    vote_tally[temp_register[vote]] = vote_tally.get(temp_register[vote],0) + 1
                rospy.logdebug(str(vote_tally))
                result = max(vote_tally.iteritems(), key=operator.itemgetter(1))[0]
                rospy.loginfo(str(result) + " <" + str(vote_tally[result]) + ">")
                pub.publish(str(result))
                pub2.publish(str(result) + " wins!  " + str(vote_seconds) + " seconds until next vote")
            next_election = rospy.Time.now() + rospy.Duration(vote_seconds)

        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
