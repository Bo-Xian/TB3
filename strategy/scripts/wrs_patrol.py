#!/usr/bin/env python
# -*- coding: utf-8 -*-+

# insert opencv 3.0 
import sys
#sys.path.insert(1,'/usr/local/lib/python3.5/dist-packages')
sys.path.insert(0,'/opt/ros/kinetic/lib/python2.7/dist-packages')
#sys.path.insert(1,'/usr/local/lib/lib/python2.7/dist-packages')


# ros lib
import rospy
import roslib
roslib.load_manifest('strategy')
import tf

import math
import numpy as np
import cv2

# rostopic msg
from std_msgs.msg import Int32
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry


# Define Behavior
INIT = 0
STRAIGHT = 1

# FLAG  
SIMULATEION_FLAG = False


def Rad2Deg(angle):
	return angle*180/math.pi

def Norm_Angle(angle):
	if(angle > 180):
		angle -=360
	elif(angle < -180):
		angle +=360
	return angle

class NodeHandle(object):
	def __init__(self):
		self._scan = None
		self._pos = None
		self._front = None

		self._start = 1
		
		if(SIMULATEION_FLAG):
			self.pub_vel = rospy.Publisher('robot1/cmd_vel',Twist, queue_size = 1)
		
			self.sub_scaninfo = rospy.Subscriber("robot1/scan",LaserScan,self.Set_ScanInfo)
			self.sub_odom = rospy.Subscriber("robot1/odom",Odometry,self.Set_Odom)
		else:
			self.pub_vel = rospy.Publisher('cmd_vel',Twist, queue_size = 1)
			
			self.sub_scaninfo = rospy.Subscriber("scan",LaserScan,self.Set_ScanInfo)
			self.sub_odom = rospy.Subscriber("odom",Odometry,self.Set_Odom)
		
		
	
		self.sub_start = rospy.Subscriber("strategy/start",Int32,self.Set_Start)

	def Set_ScanInfo(self,msg):
		self._scan = msg.ranges

	def Set_Odom(self,msg):
		self._pos = [msg.pose.pose.position.x,msg.pose.pose.position.y]
		(r,p,y) = tf.transformations.euler_from_quaternion([msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w])
        
		self._front = Rad2Deg(y)
	def Set_Start(self,msg):
		self._start = msg.data

		

class Strategy(NodeHandle):
	def __init__(self):
		super(Strategy,self).__init__()
		self.behavior = INIT

		self.initpos = [[0.0,1.6],[0.15,1.6],[0.15,0],[0.55,0],[0.55,0.50],[0.55,0],[0.8,0],[0.8,1.6],[1.0,1.6],[1.0,0],[0.0,0.0]]

		self.goal = None
		self.prev_RPdis = 999

		self.state = 0
		
		self.vel_x = 0.3
		self.vel_z = 0.2
		self.error_ang = 1.0
		self.error_dis = 0.05

		
	def Process(self):
		if(self._start):
			print(self.state,self.behavior,self.goal)
			if(self.behavior == INIT):
				self.goal = self.initpos.pop(0)
				self.behavior = STRAIGHT
			elif(self.behavior == STRAIGHT):
 				self.Straight_Strategy()
			else:
				self.Robot_Stop()		
		else:
			self.Robot_Stop()
			print('stop')

	def Robot_Vel(self,vel):
		twist = Twist()
		twist.linear.x = vel[0]
		twist.angular.z = vel[1]

		self.pub_vel.publish(twist)
	
	def Robot_Stop(self):
		twist = Twist()
		twist.linear.x = 0
		twist.angular.z = 0
        
		self.pub_vel.publish(twist)   
	
    	# robot to pose angle
	def Get_RP_Angle(self,pos):
		return Rad2Deg(math.atan2((pos[1]-self._pos[1]),(pos[0]-self._pos[0])))
	
	# robot to pose distance
	def Get_RP_Dis(self,pos):
		return math.sqrt(pow(pos[0]-self._pos[0],2.0)+pow(pos[1]-self._pos[1],2.0))
	
	# strategy
	def Straight_Strategy(self):
		ang = 0
		if(self.state == 0):
			RPang = Norm_Angle(self.Get_RP_Angle(self.goal)-self._front)
			ang = RPang			
			if(abs(RPang) > self.error_ang):
				if(RPang > 0):
			 		x = 0
					z = self.vel_z
				else:
					x = 0
					z = -self.vel_z
				self.Robot_Vel([x,z])
			else:
				self.Robot_Stop()
				self.state = 1
		elif(self.state == 1):
			RPdis = self.Get_RP_Dis(self.goal)
			RPang = Norm_Angle(self.Get_RP_Angle(self.goal)-self._front)
			
			if(RPdis > self.error_dis):
				if(self.prev_RPdis >= RPdis):
					if(abs(RPang) > self.error_ang):
						if(RPang > 0):
					 		x = self.vel_x
							z = self.vel_z
						else:
							x = self.vel_x
							z = -self.vel_z 
					else:
						x = self.vel_x
						z = 0			
				else:
					x = 0
					z = 0
					self.state = 0
				self.Robot_Vel([x,z])
				self.prev_RPdis = RPdis
			else:
				if(len(self.initpos) == 0):
					self.Robot_Stop()
				else:
					self.goal = self.initpos.pop(0)
					self.prev_RPdis = 999
					self.Robot_Stop()
					self.state = 0
					
			
		else:
			self.Robot_Stop()		
			


def main():
	rospy.init_node('strategy', anonymous=True)
	strategy = Strategy()

	# 30 hz
	rate = rospy.Rate(30)
	i = 0
	while not rospy.is_shutdown():
		if(strategy._pos):        
			strategy.Process()
		rate.sleep()
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting down")

if __name__ == "__main__":
	main()
