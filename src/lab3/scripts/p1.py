#!/usr/bin/env python3
import rospy
from std_msgs.msg import Int64
from lab3.srv import Att,AttResponse
import actionlib
from lab3.msg import AssiAction, AssiFeedback, AssiResult
global Action_Server
def first():
    global Action_Server
    rospy.init_node('first')
    pub = rospy.Publisher('students', Int64, queue_size=10)
    rate = rospy.Rate(1) # 1hz
    service = rospy.Service('Attendance', Att, att_done)
    Action_Server = actionlib.SimpleActionServer("Assignments", AssiAction, callback, auto_start=False)
    Action_Server.start()
    while not rospy.is_shutdown():
        for i in range(1,15+1):
            rate.sleep()
            pub.publish(i)
        while(1):
            pass
def att_done(req):
    return AttResponse("Attendance enrolled: "+str(req.numb)+" Seniors attended week "+str(req.week))
def callback(goal):
    global Action_Server
    feedback = AssiFeedback()
    result = AssiResult()
    rate = rospy.Rate(1)
    for i in range(1, goal.num_assi+1):
        feedback.assi_solved = i
        Action_Server.publish_feedback(feedback)
        rate.sleep()
    result.result = "Assignments Done!!"
    Action_Server.set_succeeded(result)
if __name__ == '__main__':
    first()

