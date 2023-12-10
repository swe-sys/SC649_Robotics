#!/usr/bin/env python3
import rospy
import numpy as np
from geometry_msgs.msg import Point, Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from math import atan2
import matplotlib.pyplot as plt

x = 0.0
y = 0.0 
theta = 0.0

def newOdom(msg):
	global x
	global y
	global theta
	x = msg.pose.pose.position.x
	y = msg.pose.pose.position.y
	rot_q = msg.pose.pose.orientation
	(roll, pitch, theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])
    
if __name__ == '__main__':
	k=0
	l=[0] #l is time
	x1=[0]#x-coordinate of robot
	x2=[0]#x set point
	y1=[0]#y -coordinate of robot
	y2=[0]#y set point
	theta1 =[0]
	dis_err = [0]
	u1 =[0]
	rospy.init_node("speed_controller")
	sub = rospy.Subscriber("/odom", Odometry, newOdom)
	pub = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)
	speed = Twist()
	r = rospy.Rate(4)
	goal = Point()
		
	while not rospy.is_shutdown() and k < 500:
		k = k+1
		h = 0.25
		K = 0.1
		goal.x = 5
		goal.y = 5
		inc_x = goal.x -x
		inc_y = goal.y -y
		l.append((k+1)/10)
		x1.append(x)
		y1.append(y)
		theta1.append(theta)
		x2.append(goal.x)
		y2.append(goal.y)
		angle_to_goal = atan2(inc_y, inc_x)
		dis_err = (inc_x**2+inc_y**2)**0.5
				
		if abs(angle_to_goal-theta) > 0.2:
			speed.linear.x = 0.0
			speed.angular.z = 0.12
			print('1')
		else:
			speed.linear.x = 0.18
			speed.angular.z = 0.0
			print('2')
		pub.publish(speed)
		r.sleep()

plt.figure(1)
plt.plot(x1,y1 , label='Robot position')
plt.plot(x2,y2, label='Goal')
plt.xlabel('x co-ordinate')
plt.ylabel('y co-ordinate')
plt.legend()
plt.show()