#! /usr/bin/env python
import rospy
import actionlib
import time
from std_msgs.msg import Empty
from actions_quiz.msg import CustomActionMsgFeedback, CustomActionMsgResult, CustomActionMsgAction

class Actions_quiz(object):
    
  # create messages that are used to publish feedback/result
  _feedback = CustomActionMsgFeedback()
  _result   = CustomActionMsgResult()
  def __init__(self):
    # creates the action server
    self._as = actionlib.SimpleActionServer("/action_custom_msg_as", CustomActionMsgAction, self.goal_callback, False)
    self._as.start()
    
  def goal_callback(self, goal):
    # this callback is called when the action server is called.
    # this is the function that computes the Fibonacci sequence
    # and returns the sequence to the node that called the action server
    
    # helper variables
    success = True
    _goal = goal.goal
    success = True
    r = rospy.Rate(1)
    launch = rospy.Publisher('/drone/takeoff', Empty, queue_size = 1)
    land = rospy.Publisher('/drone/land', Empty, queue_size = 1)
    rospy.loginfo('Instruction : %s' % _goal)
    time.sleep(1)
    # check that preempt (cancelation) has not been requested by the action client
    if self._as.is_preempt_requested():
        rospy.loginfo('The goal has been cancelled/preempted')
        # the following line, sets the client in preempted state (goal cancelled)
        self._as.set_preempted()
        success = False


    if (_goal == "TAKEOFF"):
        launch.publish(Empty())
        self._feedback.feedback = "TAKEOFF"
    elif (_goal == "LAND"):
        land.publish(Empty())
        self._feedback.feedback = "LAND"

    self._as.publish_feedback(self._feedback)

    
    # at this point, either the goal has been achieved (success==true)
    # or the client preempted the goal (success==false)
    # If success, then we publish the final result
    # If not success, we do not publish anything in the result
    #if success:
    #    rospy.loginfo('Successfully done command : %s' % goal.goal )
    self._as.set_succeeded(self._result)

if __name__ == '__main__':
    rospy.init_node('Actions_Quiz')
    Actions_quiz()
    rospy.spin()