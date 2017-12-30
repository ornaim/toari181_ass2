#!/usr/bin/env python
import rospy
import sys
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from math import radians
from math import pi
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
import message_filters
import time
from tf.transformations import euler_from_quaternion
import tf


range_head = -1
driving_forward = False
turning = False
lower_red = np.array([0,50,500])
upper_red = np.array([10,255,255])

def move_fowrd(data):
	global range_head
	global driving_forward 
	global closest_obj
	global sub
	global NodeInCallback 

	distance = 0.5
	closest_wall=data.ranges[len(data.ranges)/2]

	msg = Twist()
	if (not driving_forward):
		closest_obj=min(data.ranges)
		if(closest_obj > distance):
			driving_forward = True
			range_head = closest_wall - distance 
			msg.linear.x = 0.1
		else:
			msg.linear.x = 0.0
			sub.unregister()
			NodeInCallback=False
			print "obsticle ahead."
	else:
		if(closest_wall < range_head):
			msg.linear.x = 0.0
			driving_forward=False
			sub.unregister()
			NodeInCallback=False
			print "finished moving."
			
		else:
			msg.linear.x = 0.1
	
	pub.publish(msg)
	rospy.loginfo("current location:"+ str(closest_wall) + " where to Stop:" + str(range_head))

def action1():
	global sub
	global NodeInCallback 
	global closest_obj
	closest_obj = 	sys.maxint
	NodeInCallback = True
	sub= rospy.Subscriber("/scan", LaserScan, move_fowrd)
	while (NodeInCallback):
		time.sleep(0.1)


#==========================================#
#= Turn robot around in degrees. use yaw. =#
#==========================================#
def turn_around(msg):
	global NodeInCallback
	global sub
	global resultAngleR
	global turning
	global prev_yaw
	global alreadyMoved
	moveMsg = Twist()
	
	explicit_quat = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
	roll, pitch,current_yaw = tf.transformations.euler_from_quaternion( explicit_quat)
	if (not turning): 
		prev_yaw =current_yaw
	 	turning = True
	if (abs(current_yaw - prev_yaw) > 0.2):
		alreadyMoved = alreadyMoved + abs(current_yaw + prev_yaw)
	else:
		alreadyMoved = alreadyMoved + abs(current_yaw - prev_yaw)
	#print "prev_yaw: "+ str(prev_yaw) + " current_yaw: " + str(current_yaw)
	
	if (resultAngleR -alreadyMoved > radians (25) ):
		moveMsg.angular.z =-0.2
		pub.publish(moveMsg)

	elif (resultAngleR -alreadyMoved > radians (20) ):
		moveMsg.angular.z =-0.15
		pub.publish(moveMsg)

	elif (resultAngleR - alreadyMoved > radians(15)):
		#print "(resultAngleR - alreadyMoved > radians(15)"
		moveMsg.angular.z =-0.1
		pub.publish(moveMsg)
	elif (resultAngleR - alreadyMoved > radians(10)):
		#print "(resultAngleR - alreadyMoved > radians(10)"
		moveMsg.angular.z =-0.08
		pub.publish(moveMsg)	
	elif (resultAngleR - alreadyMoved > radians(4)):
		#print "(resultAngleR - alreadyMoved > radians(6)"
		moveMsg.angular.z =-0.03
		pub.publish(moveMsg)
	# elif (resultAngleR -alreadyMoved  > 0):
	# 	print "resultAngleR -alreadyMoved  > 0"
	# 	moveMsg.angular.z =-0.02
	# 	pub.publish(moveMsg)
		
	else:
		moveMsg.angular.z =0.0
		#print "speed is zero! STOPPPPPPPPP"
		pub.publish(moveMsg)
		NodeInCallback= False
		sub.unregister()
	prev_yaw = current_yaw

	print "Goal is: "+str(resultAngleR)+ " robot angle: "+str(alreadyMoved)

def action2():
	global sub
	global angle
	global NodeInCallback 
	global resultAngleR
	global alreadyMoved
	alreadyMoved = 0 
	NodeInCallback = True
	angle=float(raw_input('Enter angle in degrees: '))
	resultAngleR= radians(angle)
	sub = rospy.Subscriber("/odometry/filtered",Odometry,turn_around)

	while (NodeInCallback):
		time.sleep(0.1)	


def distance_to_red_object(img_recived,img_depth):
	print "where are you little red?"
	global minDistance 
	global redExist
	global NodeInCallback

	redExist = False
	minDistance = sys.maxint
	bridge = CvBridge()
	cv_image = bridge.imgmsg_to_cv2(img_recived, desired_encoding='bgr8')
	res_image,redExist= find_ball(cv_image)
	
	#if redExist :
	# 	show_img(cv_image)
	#	show_img(res_image)

	print redExist

	if redExist:
		cv_image_depth = bridge.imgmsg_to_cv2(img_depth , desired_encoding = 'passthrough')
		height, width, bla = res_image.shape
		for x in range(0, height):
			for y in range (0 , width):
				if res_image[x,y,0] != 0 :
					#print cv_image_depth[x,y]
				 	distance = cv_image_depth[x,y]
				 	if minDistance> distance :	
				 		minDistance = distance
	
		print minDistance
	else :
		print "NULL"
	if minDistance == sys.maxint :	
		minDistance = "NULL"


	#Calc distance to red object
	# print res_image
	depth_sub.unregister()
	sub.unregister()
	NodeInCallback= False
	return


def action3():
	global ts
	global sub
	global depth_sub
	global NodeInCallback 	
	NodeInCallback = True
	print "Looking for red object ..."
	sub = message_filters.Subscriber("/torso_camera/rgb/image_raw", Image)
	depth_sub = message_filters.Subscriber("/torso_camera/depth_registered/image_raw", Image)
	ts = message_filters.ApproximateTimeSynchronizer([sub, depth_sub], 10, 0.5)
	ts.registerCallback(distance_to_red_object)
	while (NodeInCallback):
		#print "going to sleep"
		time.sleep(0.1)
	print "finished action 3"	


def find_ball(cv_image):
	# red = np.uint8([[[0,0,255 ]]])
	# hsv_red = cv2.cvtColor(red,cv2.COLOR_BGR2HSV)
	# print "hsv values" 
	# print hsv_red
	# print "enddddddddddd"

	#print "In find_apple function"
	hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
	# lower_red = np.array([0,50,50])
	# upper_red = np.array([10,200,200])
	# mask0 =cv2.inRange(hsv,lower_red,upper_red)

	#show_img(res)
	# upper mask (170-180)


	lower_red = np.array([170,70,70])

	upper_red = np.array([180,255,255])
	mask = cv2.inRange(hsv, lower_red, upper_red)
	# join my masks
	# mask = mask0+mask1
	mask_sum=cv2.sumElems(mask)
	rospy.loginfo("mask_sum: "+ str(mask_sum));
	res = cv2.bitwise_and(cv_image,cv_image,mask=mask)
	#show_img(cv_image)
	# if(mask_sum[0]==0):
	# 	print "There is no red object!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"

	gray = cv2.cvtColor(res, cv2.COLOR_BGR2GRAY)
#	show_img(gray)

	blurred = cv2.GaussianBlur(gray, (5, 5), 0)
#	show_img(blurred)

	thresh = cv2.threshold(blurred, 20, 255, 0)[1]	
#	thresh = cv2.threshold(blurred, 10, 255, 0)[1]	
#	show_img(thresh)
	contours , hierarchy = cv2.findContours(thresh, cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
	#find best center points.
	max_area = 0
	best_cnt = 1
	for cnt in contours:
		area = cv2.contourArea(cnt)
		if area > max_area:
			max_area = area
			best_cnt = cnt

	print "max area is: " + str(max_area)

	return res,(max_area > 100)


def find_red_object():
	global depth_sub
	global sub
	global degree
	global NodeInCallback 	
	print "welcome"
	global resultAngleR
	global alreadyMoved
	degree = 0
	angle = 30
	resultAngleR= radians(angle)

	while degree <= 360 :
		#print "LOOK HERE!!!!							"+ str(degree)
		#print "spining yayyyy"
		action3()
		if minDistance != "NULL":
		# 	print minDistance;
			return minDistance
		#return minDistance
	#ACTION 2
		
		alreadyMoved = 0 
		NodeInCallback = True	
		sub = rospy.Subscriber("/odometry/filtered",Odometry,turn_around)
		degree = degree + angle
		while (NodeInCallback):
			time.sleep(1)	
	return "NULL"

def action4():
	NodeInCallback = True
	dis=find_red_object()
	print "The distance is :"+ str(dis)
	#sub= rospy.Subscriber("/scan", LaserScan, action4)


def show_img(res_image):
	print "In show_img function"
	cv2.namedWindow('Display apple')
	cv2.imshow("apple" ,res_image)
	cv2.waitKey(0)
	cv2.destroyAllWindows()	


def main():
	global NodeInCallback 
	NodeInCallback = False
	while not rospy.is_shutdown():
		commandNumber = raw_input("Enter command number: ")

		if(commandNumber == "1"):action1()
		elif(commandNumber == "2"): action2()
		elif(commandNumber == "3"): action3()
		elif(commandNumber == "4"): action4()
		elif (commandNumber == "0"):
			print "Finished bye bye"
			rospy.signal_shutdown("Finished");
			#break;
		else:
			print "Invalid Input please try again."
		

if __name__ == '__main__':
	global pub
	rospy.init_node('checkObstacle', anonymous=True)
	pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
	main()

