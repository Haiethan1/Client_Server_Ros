#! /usr/bin/env python
import rospy
import time
import actionlib
from geometry_msgs.msg import Twist
from actionlib.msg import TestFeedback, TestResult, TestAction

class FibonacciClass(object):
    
  # create messages that are used to publish feedback/result
  _feedback = TestFeedback()
  _result   = TestResult()
  def __init__(self):
    # creates the action server
    self._as = actionlib.SimpleActionServer("test_as", TestAction, self.goal_callback, False)
    self._as.start()
    
  def goal_callback(self, goal):
    # this callback is called when the action server is called.
    # this is the function that computes the Fibonacci sequence
    # and returns the sequence to the node that called the action server
    
    # helper variables
    _pub = rospy.Publisher('/cmd_vel', Twist, queue_size=2)
    _cmd = Twist()
    success = True
    
    # append the seeds for the fibonacci sequence
    #self._feedback.sequence = []
    #self._feedback.sequence.append(0)
    #self._feedback.sequence.append(1)
    self._feedback.feedback = 0
    
    # publish info to the console for the user
    rospy.loginfo('"test_as": Executing, creating test of size %i on side %i' % ( goal.goal, self._feedback.feedback))
    
    # starts calculating the Fibonacci sequence
    squareSides = goal.goal
    r = rospy.Rate(1)
    for i in xrange(1, 5):
    
      # check that preempt (cancelation) has not been requested by the action client
        if self._as.is_preempt_requested():
            rospy.loginfo('The goal has been cancelled/preempted')
            # the following line, sets the client in preempted state (goal cancelled)
            self._as.set_preempted()
            success = False
            # we end the calculation of the Fibonacci sequence
            break
        if (i == 1):
            _cmd.linear.x = 1.0
            _cmd.linear.y = 0.0
            _pub.publish(_cmd)
            time.sleep(squareSides)
        elif (i == 2):
            _cmd.linear.x = 0.0
            _cmd.linear.y = 1.0
            _pub.publish(_cmd)
            time.sleep(squareSides)
        elif (i == 3):
            _cmd.linear.x = -1.0
            _cmd.linear.y = 0.0
            _pub.publish(_cmd)
            time.sleep(squareSides)
        elif (i == 4):
            _cmd.linear.x = 0.0
            _cmd.linear.y = -1.0
            _pub.publish(_cmd)
            time.sleep(squareSides)
            _cmd.linear.x = 0
            _cmd.linear.y = 0
            _pub.publish(_cmd)
            time.sleep(squareSides)
        # builds the next feedback msg to be sent
        self._feedback.feedback += 1
        # publish the feedback
        self._as.publish_feedback(self._feedback)
        # the sequence is computed at 1 Hz frequency

    
    # at this point, either the goal has been achieved (success==true)
    # or the client preempted the goal (success==false)
    # If success, then we publish the final result
    # If not success, we do not publish anything in the result
    if success:
        self._result.result = squareSides * 4
        rospy.loginfo('Succeeded calculating the test square of length %i' % squareSides )
        self._as.set_succeeded(self._result)
      
if __name__ == '__main__':
    rospy.init_node('test')
    FibonacciClass()
    rospy.spin()