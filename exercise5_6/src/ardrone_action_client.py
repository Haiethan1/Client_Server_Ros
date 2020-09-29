#! /usr/bin/env python
import rospy
import time
import actionlib
from geometry_msgs.msg import Twist
from ardrone_as.msg import ArdroneAction, ArdroneGoal, ArdroneResult, ArdroneFeedback

"""
class SimpleGoalState:
    PENDING = 0
    ACTIVE = 1
    DONE = 2
    WARN = 3
    ERROR = 4

"""
# We create some constants with the corresponing vaules from the SimpleGoalState class
PENDING = 0
ACTIVE = 1
DONE = 2
WARN = 3
ERROR = 4
nImage = 1

# definition of the feedback callback. This will be called when feedback
# is received from the action server
# it just prints a message indicating a new message has been received
def feedback_callback(feedback):
    global nImage
    print('[Feedback] image n.%d received'%nImage)
    nImage += 1
    cmd.linear.x = 1.0
    pub.publish(cmd)
    rate.sleep()

    

# initializes the action client node
rospy.init_node('drone_action_client')

pub = rospy.Publisher('/cmd_vel', Twist, queue_size=2)
cmd = Twist()
# create the connection to the action server
action_server_name = '/ardrone_action_server'
client = actionlib.SimpleActionClient('/ardrone_action_server', ArdroneAction)
# waits until the action server is up and running
rospy.loginfo('Waiting for action Server '+action_server_name)
client.wait_for_server()
rospy.loginfo('Action Server Found...'+action_server_name)

# creates a goal to send to the action server
goal = ArdroneGoal()
goal.nseconds = 10 # indicates, take pictures along 10 seconds

# sends the goal to the action server, specifying which feedback function
# to call when feedback received
client.send_goal(goal, feedback_cb=feedback_callback)
state_result = client.get_state
rate = rospy.Rate(5)
rospy.loginfo("state_result: " +str(state_result))

while state_result < DONE :
    rospy.loginfo("Doing Stuff while waiting for the Server to give a result....")
    
    state_result = client.get_state()
    rospy.loginfo("state_result: "+str(state_result))
rospy.loginfo("[Result] State: "+str(state_result))
cmd.linear.x = 0
pub.publish(cmd)
if state_result == ERROR:
    rospy.logerr("Something went wrong in the Server Side")
if state_result == WARN:
    rospy.logwarn("There is a warning in the Server Side")
# Uncomment these lines to test goal preemption:
#time.sleep(3.0)
#client.cancel_goal()  # would cancel the goal 3 seconds after starting

# wait until the result is obtained
# you can do other stuff here instead of waiting
# and check for status from time to time 
# status = client.get_state()
# check the client API link below for more info

client.wait_for_result()

print('[Result] State: %d'%(client.get_state()))