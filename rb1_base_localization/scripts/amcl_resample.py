#!/usr/bin/python

import rospy
import geometry_msgs.msg

rospy.init_node('amcl_resample', anonymous=True)
pub = None
rate = rospy.Duration(5)
last = rospy.Time.now()

def callback(msg):
    global last
    if rospy.Time.now() - last < rate:
        return

    if pub == None:
        return

    new_msg = msg
    new_msg.pose.covariance = list(new_msg.pose.covariance)
    last = rospy.Time.now()
    for i in range(len(new_msg.pose.covariance)):
        new_msg.pose.covariance[i] /= 2.0

    pub.publish(new_msg)
    #rospy.loginfo('publico')

def listener():
    global pub

    pub = rospy.Publisher('initialpose', geometry_msgs.msg.PoseWithCovarianceStamped, queue_size=1)
    rospy.Subscriber('amcl_pose', geometry_msgs.msg.PoseWithCovarianceStamped, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
