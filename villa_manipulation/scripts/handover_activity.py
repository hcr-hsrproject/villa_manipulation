#!/usr/bin/env python
from __future__ import print_function

import hsrb_interface
from hsrb_interface import geometry
import rospy
import sys
from tmc_yolo2_ros.msg import Detections
import actionlib
from activity_detection.msg import activityDetectionAction, activityDetectionGoal

# from openpose_ros_wrapper_msgs.msg import Persons
# from openpose_ros_wrapper_msgs.msg import PersonDetection
# from openpose_ros_wrapper_msgs.msg import BodyPartDetection


def activity_detection_client():

    client = actionlib.SimpleActionClient('activity_detection', activityDetectionAction)
    
    print('client waiting for server')
    client.wait_for_server()
    
    print('client sending goal')
    goal = activityDetectionGoal()

    client.send_goal(goal)

    print('client waiting for result')
    client.wait_for_result(rospy.Duration.from_sec(5.0))

    print('Returning')
    return client.get_result()  # A FibonacciResult

def find_bag(data):
	print("Yolo data")
	bag_pose = find_bag()
	arms_pos = find_arms_poses()
	side = find_side(bag_pose,arms_pos)
	go_to_handle()
	try:
		print('trying..')
		result = activity_detection_client()
		#print("Result:", ', '.join([str(n) for n in result.sequence]))
		print('printing result..')
		print(result)
	except rospy.ROSInterruptException:
		print("program interrupted before completion", file=sys.stderr)


def find_arms_poses():
	return arms_pos

def find_side(bag_pos,arms_pos):
	return side

def go_to_handle(handle_pos):
	a=1


if __name__ == '__main__':
	print("Node initialisation")
	rospy.init_node('get_to_bag', anonymous=True)
	print("Subscribing to yolo")
	rospy.Subscriber("yolo2_node/detections", Detections, find_bag)	
	print("Subscription established")
	rospy.spin()


