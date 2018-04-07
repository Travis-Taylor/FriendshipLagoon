#!/usr/bin/env python
#
# This subscribes to a cmd_vel and publishes an twist with covariance 
# for ekf input
#
# It is basically a hack to give the EKF somethign to work with without 
# wheel encoders

import rospy
from geometry_msgs.msg import Twist, TwistWithCovarianceStamped
pub = rospy.Publisher('twist', TwistWithCovarianceStamped, queue_size=1)

def callback(data):
    t = TwistWithCovarianceStamped()
    t.twist.twist = data
    t.header.frame_id = "base_link"
    t.header.stamp = rospy.Time.now()
    t.twist.covariance[0] = 0.1
    pub.publish(t)    

def listener():
    rospy.init_node('twist_sim', anonymous=True)
    rospy.Subscriber("cmd_vel", Twist, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
