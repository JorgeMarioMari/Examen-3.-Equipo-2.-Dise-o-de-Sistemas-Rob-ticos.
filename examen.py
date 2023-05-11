#!/usr/bin/env python
import rospy 
from nav_msgs.msg import Odometry 
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist
from math import atan2
from prueba3 import stop

stop_stop = stop()

class Controller():
    def __init__(self, goal):
        self.goal = goal
        self.x = 0.0
        self.y = 0.0
        self.ze = 0.0
        self.cmd = Twist()

        self.sub = rospy.Subscriber("/odom", Odometry, self.new_odom)
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

    def new_odom(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        rot_q = msg.pose.pose.orientation
        (roll, pitch, self.ze) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])

    def run(self):
        while not rospy.is_shutdown():
            inc_x = self.goal.x - self.x
            inc_y = self.goal.y - self.y
            angle_to_goal = atan2(inc_y, inc_x)

            if abs(angle_to_goal - self.ze) > 0.1:
                self.cmd.linear.x = 0.0
                self.cmd.angular.z = 0.2
            else:
                self.cmd.linear.x = 0.5
                self.cmd.angular.z = 0.0

            self.pub.publish(self.cmd)
            
            if abs(inc_x) < 0.1 and abs(inc_y) < 0.1:
                stop_stop.stop_robot()
                break

if __name__ == '__main__':
    
    
    goals = [Point(-2, 0, 0), Point(-2, 2, 0), Point(3, 2, 0), Point(3, -1, 0), Point(1, -1, 0), Point(1, -2, 0), Point(3, -2, 0)]
    controllers = []
    for goal in goals:
        controller = Controller(goal)
        controllers.append(controller)
    
    for controller in controllers:
        controller.run()