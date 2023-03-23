#!/usr/bin/env python
from std_msgs.msg import Int64
from autonomous.srv import Attendance_enrollment,Attendance_enrollmentResponse
import rospy
def attendance():
    pub = rospy.Publisher('count', Int64, queue_size=10)
    rospy.init_node('attendance', anonymous=True)
    rate = rospy.Rate(0.5) 
    i=1
    while not rospy.is_shutdown():
        while i<16:
            rospy.loginfo("senior %d has attended " %i)
            pub.publish(i)
            i+=1
            rate.sleep()
def handle_add_two_ints(req):
    return Attendance_enrollmentResponse("attendance enrolled:%s student attended in week %s "%(a, b))
def senior_attendance_serv():
    rospy.init_node('senior_attendance_serv')
    s = rospy.Service('check_attendance', Attendance_enrollment, handle_add_two_ints)
    #print("Ready to add two ints.")
    rospy.spin()
if __name__ == "__main__":
    attendance()
    senior_attendance_serv()
