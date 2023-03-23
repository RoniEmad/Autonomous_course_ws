#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist, PoseStamped   #import msg data type "Twist" to be published
from turtlesim.msg import Pose        #import msg data type "Pose" to be subscribed
import numpy as np                    #import numpy for trignometric function, arrays... etc

from nav_msgs.msg import Odometry, Path
from tf.transformations import euler_from_quaternion, quaternion_from_euler

##Identifiy Global Variables##
#Polar coordinates 
global p
global alpha 
global beta
#Desired positions we require our robot to reach
global x_desired
global y_desired
global theta_desired
#Current positions our robot is at
global X_pos
global Y_pos
global Theta
#Control parameters
global linear_v	
global angular_v	
global Kp
global Kalpha
global Kbeta 
roll = pitch = yaw = 0.0
## Parameter Initialize ##
linear_v  = 0
angular_v = 0
Kp = 0.6
Kalpha = 1
Kbeta = -0.5
X_pos = 1
Y_pos = 1
Theta = 0

def pi_2_pi(angle):
    # a function to wrap the angle between -pi and pi (for numerical stability)
    return (angle + np.pi) % (2 * np.pi) - np.pi


def Callback (msg):

	global X_pos
	global Y_pos
	global Theta
	global roll, pitch, yaw
	orientation_q = msg.pose.pose.orientation
	orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
	(roll, pitch, yaw) = euler_from_quaternion (orientation_list)
	X_pos = round(msg.pose.pose.position.x,3)  #Get the robot's position in X
	Y_pos = round(msg.pose.pose.position.y,3)	 #Get the robot's position in Y
	Theta = yaw	#Get the robot's orientation around z
	

def Polar_Coordinates():	#Calculate polar coordinates

	global p			
	global beta
	global alpha	
	global  x_desired
	global y_desired
	global theta_desired
	global X_pos
	global Y_pos
	global Theta
	global wesel

	x_delta = float(x_desired) - float(X_pos) 
	y_delta = float(y_desired) - float(Y_pos)
  
	p = np.sqrt((np.square(x_delta))+ (np.square(y_delta))) #Calculate the distance between the desired and the current position
	if p<0.05:
		wesel=True
	gamma = np.arctan2(y_delta,x_delta)		#Calculate angle between the global X-direction of the Mobile Robot and p	

	alpha =pi_2_pi(gamma - Theta)	#Calculate angle between the local X-direction of the Mobile Robot and p
	beta = pi_2_pi(-alpha-Theta + theta_desired*np.pi/180)	#Calculate angle between p and desired global X-direction of the Mobile Robot
	print('Debug: ')
	print('p: ',p)
	print('x_delta: ',x_delta)
	print('y_delta: ',y_delta)
	print('Theta: ',Theta*180/np.pi)
	print('alpha: ',alpha*180/np.pi)
	print('beta: ',beta*180/np.pi)
	print('----------------')


def Control_Law():	#Control function to move the Mobile Robot

	global p		
	global beta
	global alpha		
	global linear_v	
	global angular_v	
	global Kp
	global Kalpha
	global Kbeta  

  	 #Calculate controlled linear velocity and angular velocity

	if np.absolute(alpha) < np.pi/2: #IF condition is incase the point is behind the turtle
		linear_v = Kp * p
	else:							 
		linear_v = -Kp * p			 #This allows the Mobile robot to move backwards with a negative velocity
	
	if linear_v>0.5:		#saturate
		linear_v=0.5
	elif linear_v<-0.5:
		linear_v=-0.5
	if abs(linear_v)<0.06:
		linear_v=abs(linear_v)/linear_v*0.06
	angular_v = Kalpha * alpha + Kbeta * beta
	print('linear_v: ',linear_v)
	print('angular_v: ',angular_v*180/np.pi)
	print('----------------')



if __name__ == '__main__':

	global x_desired
	global y_desired
	global theta_desired
	global p		
	global beta		 
	global alpha		 
	global wesel

	#Input your desired coordinates
	"""
	x_desired = float(input("Desired X goal: "))	   
	y_desired = float(input("Desired Y goal: "))
	theta_desired = float(input("Desired Theta goal: "))
	"""


	#Node, Publisher and Subscriber Setup
	rospy.init_node('Turtle_Control', anonymous=True) 					#Initialize ROS node
	path_msg=rospy.wait_for_message("/path_BFS",Path)
	print("received",path_msg.poses)
	path=list()
	for way_point in path_msg.poses:
		print(way_point.pose.position)
		
		path.append([way_point.pose.position.x,way_point.pose.position.y,way_point.pose.position.z])
	#print("this",path)
	pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)		#Publish to velocity commands to the turtle topic 
	rate = rospy.Rate(10)										# rate of publishing msg 10hz
	sub = rospy.Subscriber('/odom', Odometry, Callback)
	
	vel_msg = Twist()
	
	while not rospy.is_shutdown():
		for way_point in path:
			wesel=False
			x_desired,y_desired,theta_desired=way_point
			print("this",x_desired,y_desired,theta_desired)
			while not wesel:
				Polar_Coordinates()
				Control_Law()
						
				#Calculate the linear and angular velocities
				v = round(linear_v,2) 	#Linear Velocity
				w = round(angular_v,2)	#Angular Velocity
				
				#Set the values of the Twist msg to be published
				vel_msg.linear.x = v  #Linear Velocity
				vel_msg.linear.y = 0
				vel_msg.linear.z = 0
				vel_msg.angular.x = 0
				vel_msg.angular.y = 0
				vel_msg.angular.z = w #Angular Velocity

				#ROS Code Publisher
				pub.publish(vel_msg)	#Publish msg
				rate.sleep()			#Sleep with rate
				#Set the values of the Twist msg to be published
		vel_msg.linear.x = 0  #Linear Velocity
		vel_msg.linear.y = 0
		vel_msg.linear.z = 0
		vel_msg.angular.x = 0
		vel_msg.angular.y = 0
		vel_msg.angular.z = 0 #Angular Velocity
		#ROS Code Publisher
		pub.publish(vel_msg)	#Publish msg
		rospy.loginfo("Path Completed :)")
		break