#!/usr/bin/env python
import rospy
from std_msgs.msg import Int64
from Attendance_enrollment.srv import*
x=0
def callbackk(data):
	global x
	x=(data.data)
	rospy.loginfo("senior %d has been taken attendance", data.data)
	
	
def senior_attendance_req(student, week):
	rospy.wait_for_service('check_attendance')
		check_attendance = rospy.ServiceProxy('check_attendance', Attendance_enrollment)
		resp1 = check_attendance(student, week)
    return resp1.atnd
def senior_attendance():
	global x
	rospy.init_node('senior_attendance', anonymous=True)	
	rospy.Subscriber("count", Int64, callbackk)
	
	rospy.spin()
	
if __name__=='__main__':
	global x
	student =x
	week=4
	
	senior_attendance()
	if x==15:
		
		print("attendance enrolled:%s student attended in week %s "%(student, week, senior_attendance_req(student, week)))
