#!/usr/bin/env python3
import rospy 
import numpy as np 
import random
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler

position = np.zeros((1,3))
position_Noise = np.zeros((1,3))

# Standard Deviation Inputs
SD = [0.1,0.1,0.1]
#Callback function which is called when a new message of type Odometry is received by the subscriber
def callback(msg):
	orientation_q = msg.pose.pose.orientation
	orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
	(roll, pitch, yaw) = euler_from_quaternion (orientation_list)
	x = round(msg.pose.pose.position.x,3)
	y = round(msg.pose.pose.position.y,3)
	Theta = yaw

	global position
	global Pose_msg
	
	#Save the positions in an array
	position = [x, y, Theta] 

	Pose_msg = Pose()
	Pose_msg.x = round(position[0],4)
	Pose_msg.y = round(position[1],4)
	Pose_msg.theta = round(position[2],4)
	pub2.publish(Pose_msg)
	#Call the add noise function.
	Position_Noise = add_Noise(position)
	Pose_msg = Pose()
	Pose_msg.x = round(Position_Noise[0],4)
	Pose_msg.y = round(Position_Noise[1],4)
	Pose_msg.theta = round(Position_Noise[2],4)

	pub.publish(Pose_msg)	#Publish msg
	




def add_Noise(position):
	global SD
	global position_Noise

	x_noise = position[0] + random.uniform(-SD[0],SD[0])		#Noisy data calculated at x
	y_noise = position[1] + random.uniform(-SD[1],SD[1])		#Noisy data calculated at y
	theta_noise = position[2] + random.uniform(-SD[2],SD[2])	#Noisy data calculated at theta

	position_Noise = [x_noise, y_noise, theta_noise]   #Store the noisy position in array
 
	return(position_Noise)



if __name__ == '__main__':     # Main function that is executed 

	# Initialize ROS Node 
	rospy.init_node('Turtle_Noise', anonymous = True) #Identify Ros Node
	# Initialize The Subscriber
	sub = rospy.Subscriber('/odom', Odometry, callback)
	# Initialize The Publisher
	pub = rospy.Publisher('/Turtle_Noise', Pose, queue_size=10) #Publish the New Noisy message
	pub2 = rospy.Publisher('/Ground_Truth', Pose, queue_size=10) #Publish the New Noisy message
	rospy.spin()


