#!/usr/bin/env python3
import rospy
from std_msgs.msg import Int64
from lab3.srv import *
import actionlib
from lab3.msg import AssiAction, AssiGoal
global client
week=5
def second():
    global client
    rospy.init_node('second')
    rospy.Subscriber("students", Int64, callback)
    client = actionlib.SimpleActionClient('Assignments', AssiAction)
    rospy.spin()
def callback(data):
    global week
    print("Senior",data.data,"has attended")
    if (data.data ==15):
        att_service(data.data,week)  
def att_service(students_numb,week):
    rospy.wait_for_service('Attendance')
    notifier = rospy.ServiceProxy('Attendance', Att)
    response = notifier(students_numb,week)
    print(response.notification)
    global client
    client.wait_for_server()
    goal = AssiGoal()
    goal.num_assi = 6
    client.send_goal(goal,feedback_cb=feedback_cb)
    client.wait_for_result()
    result = client.get_result()
    print(result)
def feedback_cb(msg):
    print('Assignments Solved:', msg)
if __name__ == '__main__':
    second()
