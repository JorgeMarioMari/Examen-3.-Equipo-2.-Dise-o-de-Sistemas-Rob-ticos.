#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist, Point
import time
from nav_msgs.msg import Odometry


class stop():

    def __init__(self):
        rospy.init_node('stop', anonymous=True)
        self.vel_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.cmd = Twist()
        self.ctrl_c = False
        self.rate = rospy.Rate(1)
        rospy.on_shutdown(self.shutdownhook)

    def shutdownhook(self):
        
        self.ctrl_c = True

    def publish_once_in_cmd_vel(self):
        
        while not self.ctrl_c:
            connections = self.vel_publisher.get_num_connections()
            if connections > 0:
                self.vel_publisher.publish(self.cmd)
                
                break
            else:
                self.rate.sleep()

    def stop_robot(self):
        
        self.cmd.linear.x = 0.0
        self.cmd.angular.z = 0.0
        self.publish_once_in_cmd_vel()

if __name__ == '__main__':

    stop= stop()
    try:
        stop.stop_robot()

    except rospy.ROSInterruptException:
        pass

