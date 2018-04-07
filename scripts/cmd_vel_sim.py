#!/usr/bin/env python
#
# This produces a cmd_vel periodically
#
# it is for simulation just to test the EKF without a real robot

import rospy
from geometry_msgs.msg import Twist

pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)

def callback(data):
  cmd = Twist()
  cmd.linear.x = 0.1;
  pub.publish(cmd)    

if __name__ == '__main__':
  rospy.init_node('cmd_vel_sim', anonymous=True)
  rospy.Timer(rospy.Duration(0.1), callback)
  rospy.spin()
