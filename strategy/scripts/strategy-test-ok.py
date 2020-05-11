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
import copy
import math
import numpy as np
#import cv2
import time
from counter import TimeCounter

# rostopic msg
from std_msgs.msg import Int32
from std_msgs.msg import Empty as msg_Empty
from std_msgs.msg import Bool
from std_msgs.msg import Int32MultiArray
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from vision.msg import aim,aim2
from std_srvs.srv import Empty,EmptyResponse

# Define Behavior
INIT = 0
BEGINER = 1
WEB_GO = 2
FIND_BALL = 3
CATCH_BALL = 4
GOAL = 5
FIND_BALL2 = 6
SHORT_SHOOT = 7
FAR_SHOOT = 8

# FLAG
SIMULATEION_FLAG = False

def Rad2Deg(angle):	 #rad to deg
	return angle * 180 / math.pi
def Norm_Angle(angle):
	if(angle > 180):
		angle -= 360
	elif(angle < -180):
		angle += 360
	return angle
def zerolistmaker(n):  #Create an array element as n
    listofzeros = [0] * n
    return listofzeros
class NodeHandle(object):
	def __init__(self):
		self._pos = None
		self._front = None
		self._start = 0
		self._ballsColor = None
		self._ballsDis = None
		self._ballsAng = None
		self._scan = zerolistmaker(360)
		self._double = None
		self._object = zerolistmaker(20)
		self.Strategy_Params()
		self._goal = []
		self.find_goal_count = 0

		#test web coordinates set up
		self.web_axis = None
		self.web_set_up =False
		self.web_ang = None

		self.Timer_10 = TimeCounter(time=10.0)
		if(SIMULATEION_FLAG):
			#Simulation topic Publisher
			self.pub_vel = rospy.Publisher('robot1/cmd_vel', Twist, queue_size = 1)
			#Simulation topic Subscriber
			self.sub_scaninfo = rospy.Subscriber("robot1/scan", LaserScan, self.Set_ScanInfo)
			self.sub_odom = rospy.Subscriber("robot1/odom", Odometry, self.Set_Odom)
		else:
			#actual topic Publisher
			self.pub_vel = rospy.Publisher('cmd_vel', Twist, queue_size = 1)
			self.pub_arm = rospy.Publisher('/tb3/arm', Int32, queue_size = 1)
			self.pub_shoot = rospy.Publisher("tb3/shoot", msg_Empty, queue_size = 1)
			#actual topic Subscriber
			self.sub_scaninfo = rospy.Subscriber("scan", LaserScan, self.Set_ScanInfo)
			self.sub_odom = rospy.Subscriber("odom", Odometry, self.Set_Odom)
			self.sub_double = rospy.Subscriber("tb3/strategy/double", Int32, self.Set_Double)
			self.sub_strategy = rospy.Subscriber("tb3/strategy/strategy", Int32, self.Set_Strategy)
		#Share topic Publisher
		self.pub_moving = rospy.Publisher("tb3/strategy/moving", Bool, queue_size = 1)
		#Share topic Subscriber
		self.sub_balls = rospy.Subscriber("tb3/ball", Int32MultiArray, self.Set_Balls)
		self.sub_objects = rospy.Subscriber("tb3/object", Int32MultiArray, self.Set_Object)
		self.sub_objects2 = rospy.Subscriber("tb3/object2", Int32MultiArray, self.Set_Object2)
		self.sub_start = rospy.Subscriber("tb3/strategy/start", Int32, self.Set_Start)
		self.sub_save = rospy.Subscriber("tb3/save", Int32, self.Set_Param)
		#Service
		rospy.Service('tb3/strategy/save', Empty, self.Call_Get_Param)

		# rospy.Service('tb3/strategy/coordinate', bool, self.Set_coordinates)
		if rospy.has_param("tb3/center"):
			self.catchBallDis_param = rospy.get_param("tb3/center")
			self.catchBallDis = self.catchBallDis_param[2]
		print("ready")
	def Strategy_Params(self):
		self.strategy = 1
		self.vel_x = 0.3
		self.vel_z = 0.2
		self.error_ang = 1.0
		self.error_dis = 0.025
		self.find_ball_vel_z = 0.6
		self.goal_error_ang = 0.5
		self.slow_vel_x = 0.2
		self.avoid_error_dis = 0.3

		self.max_distance = 1.5
		self.min_distance = 0.4
		self.x_max_speed = 0.6
		self.x_min_speed = 0.2

		self.max_angle = 30
		self.min_angle = 5
		self.z_max_speed = 0.6
		self.z_min_speed = 0.2
	def Get_Strategy_Params(self):
		if rospy.has_param("tb3/strategy/robot"):
			self.robot_params = rospy.get_param("tb3/strategy/robot")
			self.vel_x = self.robot_params[0]
			self.vel_z = self.robot_params[1]
			self.error_ang = self.robot_params[2]
			self.error_dis = self.robot_params[3]
			self.find_ball_vel_z = self.robot_params[4]
			self.goal_error_ang = self.robot_params[5]
			self.slow_vel_x = self.robot_params[6]
		if rospy.has_param("tb3/strategy/s_planning"):
			self.sp_params = rospy.get_param("tb3/strategy/s_planning")
			self.max_distance = self.sp_params[0]
			self.min_distance = self.sp_params[1]
			self.x_max_speed = self.sp_params[2]
			self.x_min_speed = self.sp_params[3]
			self.max_angle = self.sp_params[4]
			self.min_angle = self.sp_params[5]
			self.z_max_speed = self.sp_params[6]
			self.z_min_speed = self.sp_params[7]
	def Call_Get_Param(self, req):
		self.Get_Strategy_Params()
		print('update parameters')
		return EmptyResponse()
	def Set_ScanInfo(self, msg):
		for i in range(len(msg.ranges)):
			if(msg.ranges[i] == 0.0):#too far
				self._scan[i] = 999
			else:#noraml
				self._scan[i] = msg.ranges[i]
	def Set_Odom(self, msg):
		self._pos = [msg.pose.pose.position.x, msg.pose.pose.position.y]
		#quaternion to euler
		(Alpha, Beta, Gamma) = tf.transformations.euler_from_quaternion([msg.pose.pose.orientation.x,\
																		msg.pose.pose.orientation.y,\
																		msg.pose.pose.orientation.z,\
																		msg.pose.pose.orientation.w])
		self._front = Rad2Deg(Gamma)
	def Set_Start(self, msg):
		self._start = msg.data
		if(not self._start):
			print('stop')
			self.Robot_Stop()
		else:
			print('start')
	def Set_Balls(self, msg):
		self._ballsColor = msg.data[0]	#0red 1blue 2yellow
		self._ballsAng = msg.data[1]
		self._ballsDis = msg.data[2]
	def Set_Object(self, msg):
		self._object = msg.data
	def Set_Object2(self, msg):
		self._goal = msg.data
	def Set_Double(self, msg):
		self._double = msg.data
		print(self.color_to_strings(self._double))
	def Set_Param(self, msg):
		self.catchBallDis_param = rospy.get_param("tb3/center")
		self.catchBallDis = self.catchBallDis_param[2]
	def Set_Strategy(self, msg):
		self.strategy = msg.data
		switcher = {
			0 : "GOAL_STRATEGY",
			1 : "SHORT_SHOOT",
			2 : "FAR_SHOOT"
		}
		print(switcher.get(self.strategy,"FUCK YOU STRATEGY"))
	def Delay(self,time_counter):
		print("")
	# def Set_coordinates(self,msg):
	# 	self.web_set_up = msg.set_up
	# 	self.web_axis = list(zip(msg.x_axis, msg.y_axis))
	# 	self.web_ang = msg.z_axis
	# 	return True

class Strategy(NodeHandle):
	def __init__(self):
		super(Strategy,self).__init__()
		self.Get_Strategy_Params()
		self.state = 0
		self.behavior = INIT
		self.initpos = None
		self.findballpos = [1.9, 0.05]
		self.goal = None
		self.goalang = None
		self._goalarea = {0:[3.1, -0.6],\
						  1:[3.1, 0.0],\
						  2:[3.1, 0.3],\
						  3:[3.1, 0.6],\
						  4:[3.1, -0.3]}
		self.prev_RPdis = 999 #RP=robot pose
		self.findang = 30
		self.ballcolor = None
		self.balldis = 999
		self.ballang = 999
		self.ballarea = 0
		self.lostball = False

		self.web_set_up = True
		self.web_axis = [[1,0.5],[1,-1]]#up = x left = y
		self.web_ang = [90,180]# right = +

		# self.web_axis = []#up = x left = y
		# self.web_ang = []
		self.counter = 0
	def behavior_to_strings(self, argument):
		switcher = {
			0 : "INIT",
			1 : "BEGINER",
			2 : "WEB_GO",
			3 : "FIND_BALL",
			4 : "CATCH_BALL",  #positive
			5 : "GOAL",
			6 : "FIND_BALL2",  #reverse
			7 : "SHORT_SHOOT",
			8 : "FAR_SHOOT"
		}
		return switcher.get(argument, "nothing")
	def color_to_strings(self, argument):
		switcher = {
			0: "Red",
			1: "Blue",
			2: "Yellow",
			3: "White",
			4: "Black",
			999:"Error",
		}
		return switcher.get(argument, "None")
	def Robot_Vel(self, vel):
		twist = Twist()
		twist.linear.x = vel[0]
		twist.angular.z = vel[1]
		self.pub_vel.publish(twist)
	def Catch_Ball(self, control):
		self.pub_arm.publish(control)
	def Robot_Stop(self):
		twist = Twist()
		twist.linear.x = 0
		twist.angular.z = 0
		self.pub_vel.publish(twist)
	def Robot_Moving(self):
		self.pub_moving.publish(True)
	def Get_RP_Angle(self, pos):  #robot to pose angle
		ang = Rad2Deg(math.atan2((pos[1] - self._pos[1]), (pos[0] - self._pos[0])))
		return ang
	def Get_RP_Dis(self, pos):# robot to pose distance
		dis = math.sqrt(pow(pos[0] - self._pos[0], 2.0) + pow(pos[1] - self._pos[1], 2.0))#p2p formula
		return dis
	def x_speed_planning(self, distance):#X linear speed conversion
		planning_speed = (math.fabs(distance) - self.min_distance)\
						 / (self.max_distance - self.min_distance)\
						 * (self.x_max_speed - self.x_min_speed)\
						 + self.x_min_speed
		if(math.fabs(distance) > self.max_distance):
			planning_speed = self.x_max_speed
		elif(math.fabs(distance) < self.min_distance):
			planning_speed = self.x_min_speed
		if(distance==0):
			planning_speed = 0
		# print (planning_speed)
		return planning_speed
	def z_speed_planning(self, angle):#Z linear speed conversionweb_ang
		planning_speed = (math.fabs(angle) - self.min_angle)\
						 / (self.max_angle - self.min_angle)\
						 * (self.z_max_speed - self.z_min_speed)\
						 + self.z_min_speed
		if(math.fabs(angle) > self.max_angle):
			planning_speed = self.z_max_speed
		elif(math.fabs(angle) < self.min_angle):
			planning_speed = self.z_min_speed
		if(angle != 0):
			planning_speed = planning_speed if(angle > 0) else -planning_speed
		else:
			planning_speed = 0
		return planning_speed
	def Avoidance_Strategy(self):#Waiting for modification
		obstacle = 0
		obstacle_num = 0
		avoidance_distance = 0.5
		avoidance_angle = 45
		avoidance_vel = 0.5
		have_obstale = False
		min_distance = 999
		min_distance_angle = 0
		for i in range(len(self._scan) - 10):#-10 debug
			if((i < 80 or i > 280) and self._scan[i] < 0.8):#left and right too close
				if(self._scan[i] < 0.45):#too close
					have_obstale = True
				elif(i > 280):#Left side
					if(math.fabs(math.sin(360 - i) * self._scan[i]) > 0.4):
						have_obstale = True
				elif (i < 80):#Right side
					if (math.fabs(math.sin(i) * self._scan[i]) > 0.4):
						have_obstale = True
			elif(i > len(self._scan)):#debug
				break
			if(self._scan[i] < avoidance_distance):#scan value less than avoidance distance
				if(self._scan[i] < min_distance):
					min_distance = self._scan[i]
					min_distance_angle = i
				if(i < avoidance_angle and i > 0): #left side
					obstacle_num += 1
					obstacle -= (avoidance_distance - self._scan[i])
				if(i > (360-avoidance_angle) or i < 0):#right side
					obstacle_num += 1
					obstacle += (avoidance_distance - self._scan[i])
		if(obstacle_num):
			obstacle = obstacle * 10 / obstacle_num
			if(obstacle > avoidance_vel):
				obstacle = avoidance_vel
			elif(obstacle < avoidance_vel * -1):
				obstacle = avoidance_vel * -1
		obstacle = obstacle * 1.2
		add_range = 20
		if(min_distance_angle < (0 + add_range) or min_distance_angle > (360 - add_range)):
			if(min_distance < 0.4):
				obstacle = obstacle * 3.5
		return obstacle, have_obstale
  	# strategy
	def Init_Strategy(self):
		print ("Init_Strategy")
		self.state = 0
		self.behavior = BEGINER
		self.initpos = [[0.1,0.0]]
		self.findballpos = [1.9,0.05]
		self.goal = self.initpos.pop(0)
		self.goalang = None
		self._goalarea = {0:[3.2,0.0],1:[3.2,-0.5],2:[3.2,0.5]}
		self.prev_RPdis = 999 #RP=robot pose
		self.vel_x = 0.4
		self.vel_z = 0.3
		self.error_ang = 2.0
		self.error_dis = 0.025
		self.findang = 30
		self.ballcolor = None
		self.balldis = 999
		self.ballang = 999
		self.ballarea = 0

		self.web_set_up = True
		self.web_axis = [[1,0.5],[1,-1]]#up = x left = y
		self.web_ang = [90,180]# right = +

		# self.web_axis = []#up = x left = y
		# self.web_ang = []

		self.Catch_Ball(0) #open
	def Beginer_Strategy(self):
		if(self.web_set_up):
			self.behavior = WEB_GO
			self.goal = self.web_axis.pop(0)
		else:
			if(self.state == 0):
				RPang = Norm_Angle(self.Get_RP_Angle(self.goal) - self._front)
				if(abs(RPang) > self.error_ang):
					z = self.vel_z if(RPang > 0) else -self.vel_z
					self.Robot_Vel([0, z])
				else:
					self.Robot_Stop()
					self.state = 1
			elif(self.state == 1):
				RPdis = self.Get_RP_Dis(self.goal)
				RPang = Norm_Angle(self.Get_RP_Angle(self.goal) - self._front)
				if(RPdis > self.error_dis):
					if(self.prev_RPdis >= RPdis):
						x = self.x_speed_planning(RPdis)
						z = self.z_speed_planning(RPang) if(abs(RPang) > self.error_ang) else 0
					else:
						x = 0
						z = 0
						if(RPdis < 0.3):# exceed
							self.state = 0
					if(RPdis > self.avoid_error_dis): # Avoidance
						_obstacle,_have_obstale = self.Avoidance_Strategy()
						if(_have_obstale == True):
							x = self.slow_vel_x
							z = z * 0.8 + _obstacle
						else:
							z += _obstacle
					self.Robot_Vel([x, z])
					self.prev_RPdis = RPdis
				else:
					if(len(self.initpos) == 0):
						self.behavior = FIND_BALL2
					else:
						self.goal = self.initpos.pop(0)
						self.prev_RPdis = 999
					self.state = 0
					self.Robot_Stop()
			else:
				self.Robot_Stop()
	def Find_Ball_Strategy(self):
		if(self.state == 0):
			if(self.lostball == False):
				self.ballcolor = None
				self.balldis = 999
				self.ballang = 999
			RPang = 90 + self.findang - self._front if(self._front >= 0) else -90 + self.findang - self._front
			RPang = Norm_Angle(RPang)
			if(abs(RPang) > self.error_ang):
				z = self.find_ball_vel_z if(RPang > 0) else -self.find_ball_vel_z
				self.Robot_Vel([0, z])
			else:
				self.Robot_Stop()
				self.state = 1 if(self._front >= 0) else 2
		elif(self.state == 1 or self.state == 2):
			RPang = -90 + self.findang - self._front if(self.state == 1) else 90 + self.findang - self._front
			RPang = Norm_Angle(RPang)
			if(abs(RPang) > self.error_ang):
				z = self.find_ball_vel_z if(RPang > 0) else -self.find_ball_vel_z
				self.Robot_Vel([0, z])
				if(self.lostball == True):
					if(self._ballsColor == self.ballcolor and self._ballsDis < 250):
						self.ballcolor = self._ballsColor
						self.balldis = self._ballsDis
						self.ballang = self._front
						print('find ball')
						self.Robot_Stop()
						self.behavior = CATCH_BALL
						self.state = 0
						self.lostball = False
				else:
					if(self._ballsDis < self.balldis):
						self.ballcolor = self._ballsColor
						self.balldis = self._ballsDis
						self.ballang = self._front
					if(self._ballsColor == self._double and self._ballsDis < 280 and self.balldis > 200):
						self.ballcolor = self._ballsColor
						self.balldis = self._ballsDis
						self.ballang = self._front
						print("double!!!!!!!!")
						self.Robot_Stop()
						self.behavior = CATCH_BALL
						self.state = 0
						self.lostball = False
			else:
				self.Robot_Stop()
				self.lostball = False
				self.behavior = CATCH_BALL
				self.state = 0
	def Find_Ball_Strategy2(self):
		if(self.state == 0):
			if(self.lostball == False):
				self.ballcolor = None
				self.balldis = 999
				self.ballang = 999
			RPang = self.findang - self._front if(self._front >= 0) else -self.findang - self._front
			RPang = Norm_Angle(RPang)
			if(abs(RPang) > self.error_ang):
				z = self.find_ball_vel_z if(RPang > 0) else -self.find_ball_vel_z
				self.Robot_Vel([0, z])
			else:
				self.Robot_Stop()
				self.state = 1 if(self._front >= 0) else 2
		elif(self.state == 1 or self.state == 2):
			RPang = -self.findang - self._front if(self.state == 1) else self.findang - self._front
			RPang = Norm_Angle(RPang)
			if(abs(RPang) > self.error_ang):
				z = self.find_ball_vel_z if(RPang > 0) else -self.find_ball_vel_z
				self.Robot_Vel([0, z])
				if(self.lostball == True):
					if(self._ballsColor == self.ballcolor and self._ballsDis<250):
						self.ballcolor = self._ballsColor
						self.balldis = self._ballsDis
						self.ballang = self._front
						print('find ball')
						self.Robot_Stop()
						self.behavior = CATCH_BALL
						self.state = 0
						self.lostball = False
				else:
					if(self._ballsDis < self.balldis):
						self.ballcolor = self._ballsColor
						self.balldis = self._ballsDis
						self.ballang = self._front
					if(self._ballsColor == self._double and self.balldis > 200 and self._ballsDis < 280):
						self.ballcolor = self._ballsColor
						self.balldis = self._ballsDis
						self.ballang = self._front
						print("double!!!!!!!!")
						self.Robot_Stop()
						self.behavior = CATCH_BALL
						self.state = 0
						self.lostball = False
			else:
				self.Robot_Stop()
				self.lostball = False
				self.behavior = CATCH_BALL
				self.state = 0
				self.lostball = False
	def Catch_Ball_Strategy(self):
		self.lostball = False
		if(self.state == 0):
			if(self.ballang == 999):
				self.Robot_Stop()
				self.behavior = FIND_BALL if(self._pos[0] > 1.0) else FIND_BALL2
				self.state = 0
				self.ballcolor = None
				self.balldis = 999
				self.ballang = 999
			else:
				RBang = Norm_Angle(self.ballang - self._front)
				if(abs(RBang) > 2):
					z = self.vel_z if(RBang > 0) else -self.vel_z
					self.Robot_Vel([0, z])
				else:
					self.Robot_Stop()
					self.state = 1
		elif(self.state == 1):
			color = self.ballcolor
			if(self._ballsColor == self.ballcolor):
				color = self._ballsColor
			if(color != None):
				RBang = self._ballsAng
				if(abs(RBang) > self.error_ang):
					z = self.vel_z if(RBang > self.error_ang) else -self.vel_z
					self.Robot_Vel([0, z])
				else:
					self.Robot_Stop()
					self.state = 2
			else:
				self.Robot_Stop()
				self.state = 0
				self.behavior = FIND_BALL if(self._pos[0] > 1.0) else FIND_BALL2
				self.ballcolor = None
				self.ballang = 999
				self.ballarea = 0
		elif(self.state == 2):
			color = None
			if(self.ballcolor != None):
				if(self._object[self.ballcolor * 4 + 1] < 999):
					color = self.ballcolor
			if(color != None):
				RPdis = self._object[self.ballcolor * 4 + 1]
				RBang = -self._object[self.ballcolor * 4]
				if(RPdis > self.catchBallDis):
					if(self.prev_RPdis >= RPdis):
						x = self.vel_x if(RPdis > 220) else self.slow_vel_x
						if(abs(RBang) > self.error_ang):
							z = self.vel_z if(RBang > 0) else -self.vel_z
						else:
							z = 0
					else:
						x = 0
						z = 0
						if(RPdis < 0.3):# exceed
							self.state = 1
					if(RPdis > 220):
						_obstacle,_have_obstale = self.Avoidance_Strategy()
						if(_have_obstale == True):
							x = self.slow_vel_x
						z += _obstacle
					self.Robot_Vel([x, z])
					self.prev_RPdis = RPdis
				else:
					x = 0.05
					self.Catch_Ball(1)
					switcher = {
						0 : GOAL,
						1 : SHORT_SHOOT,
						2 : FAR_SHOOT
					}
					self.behavior = switcher.get(self.strategy,GOAL)
					self.goal = self._goalarea[color]
					self.state = 0
					self.Robot_Stop()
			else:
				self.state = 0
				self.lostball = True
				print('lostball')
				self.behavior = FIND_BALL if(self._pos[0] > 1.5) else FIND_BALL2
				self.ballang = 999
				self.ballarea = 0
				self.Robot_Stop()
	def	Goal_Strategy(self):
		front_goal = copy.deepcopy(self.goal)
		front_goal[0] = 2.5
		#=======================go to front of goal area===================
		if(self.state == 0):
			RPang = Norm_Angle(self.Get_RP_Angle(front_goal) - self._front)
			if(abs(RPang) > self.error_ang):
				z = self.find_ball_vel_z if(RPang > 0) else -self.find_ball_vel_z
				self.Robot_Vel([0, z])
			else:
				self.Robot_Stop()
				self.state = 1
		elif(self.state == 1):
			RPdis = self.Get_RP_Dis(front_goal)
			RPang = Norm_Angle(self.Get_RP_Angle(front_goal) - self._front)
			if(RPdis > self.error_dis):
				if(self.prev_RPdis >= RPdis):
					x = self.x_speed_planning(RPdis)
 					z = self.z_speed_planning(RPang) if(abs(RPang) > self.error_ang) else 0
				else:
					x = 0
					z = 0
					if(RPdis < self.error_dis):
						self.state = 0
				if(RPdis > self.avoid_error_dis):
					_obstacle,_have_obstale = self.Avoidance_Strategy()
					if(_have_obstale == True):
						x = self.slow_vel_x
					z += _obstacle
				self.Robot_Vel([x, z])
				self.prev_RPdis = RPdis
			else:
				x = -self.vel_x
				z = 0
				self.Robot_Vel([x, z])
				self.state = 2
		#=======================go to goal area===================
		elif(self.state == 2):
			RPang = Norm_Angle(self.Get_RP_Angle(self.goal) - self._front)
			if(abs(RPang) > self.goal_error_ang):
				z = self.z_speed_planning(RPang)
				self.Robot_Vel([0, z])
			else:
				self.Robot_Stop()
				z = 0
				time.sleep(1)
				self.state = 3
		elif(self.state == 3):
			RPdis = self.Get_RP_Dis(self.goal)
			RPang = Norm_Angle(self.Get_RP_Angle(self.goal) - self._front)
			print('self.goal',self.goal)
			print('RPdis',RPdis)
			if(RPdis > self.error_dis):
				if(self.prev_RPdis >= RPdis):
					x = self.x_speed_planning(RPdis)
					z = self.z_speed_planning(RPang) if(abs(RPang) > self.error_ang) else 0
				else:
					x = 0
					z = 0
					if(RPdis < self.error_dis):
						self.state = 0
				if(RPdis > self.avoid_error_dis):
					_obstacle,_have_obstale = self.Avoidance_Strategy()
					if(_have_obstale == True):
						x = self.slow_vel_x
					z += _obstacle

				self.Robot_Vel([x, z])
				self.prev_RPdis = RPdis
			else:
				self.Robot_Stop()
				self.Catch_Ball(0)
				self.state = 4
		elif(self.state == 4):
			RPdis = self.Get_RP_Dis([2.7,self.goal[1]])
			RPang = Norm_Angle(180 - (self.Get_RP_Angle([2.7,self.goal[1]]) - self._front))
			print('RPang',RPang)
			if(abs(RPang) > self.error_ang):
				z = -self.vel_z if(RPang > 0) else self.vel_z
				self.Robot_Vel([0, z])
			else:
				self.Robot_Stop()
				self.state = 5
		elif(self.state == 5):
			RPdis = self.Get_RP_Dis([2.7,self.goal[1]])
			RPang = Norm_Angle(180 - (self.Get_RP_Angle([2.7,self.goal[1]]) - self._front))
			if(RPdis > self.error_dis):
				if(self.prev_RPdis >= RPdis):
					x = -self.vel_x
					if(abs(RPang) > self.error_ang):
						z = -self.vel_z if(RPang > 0) else self.vel_z
					else:
						z = 0
				else:
					x = 0
					z = 0
					self.state = 4
				self.Robot_Vel([x, z])
				self.prev_RPdis = RPdis
			else:
				self.Robot_Stop()
				print("finish")
				self.lostball = False
				self.Robot_Stop()
				self.state = 0
				self.behavior = FIND_BALL
				self.ballcolor = None
				self.balldis = 999
				self.ballang = 999
				self.ballarea = 0
				self.goal = self.findballpos
		else:
			self.Robot_Stop()
	def	Short_Shoot_Strategy(self):
		front_goal = copy.deepcopy(self.goal)
		front_goal[0] = 2.5
		goal_offset = self._goal[self.ballcolor * 4]
		goal_offset_error = 30
		goal_size = self._goal[self.ballcolor * 4 + 2]
		goal_size_filter = 3000
		if(self.state == 0):
			RPang = Norm_Angle(self.Get_RP_Angle(front_goal) - self._front)
			if(abs(RPang) > self.error_ang):
				z= self.find_ball_vel_z if(RPang > 0) else -self.find_ball_vel_z
				self.Robot_Vel([0, z])
			else:
				self.Robot_Stop()
				self.state = 1
		elif(self.state == 1):
			RPdis = self.Get_RP_Dis(front_goal)
			RPang = Norm_Angle(self.Get_RP_Angle(front_goal) - self._front)
			if(self._pos[0] > front_goal[0]):
				self.Robot_Stop()
				self.state = 2 if(self._front >= 0) else 3
			if(RPdis > self.error_dis):
				if(self.prev_RPdis >= RPdis):
					x = self.x_speed_planning(RPdis)
					z = self.z_speed_planning(RPang) if(abs(RPang) > self.error_ang) else 0
				else:
					x = 0
					z = 0
					if(RPdis < self.error_dis):
						self.state = 0
				if(RPdis > self.avoid_error_dis):
					_obstacle,_have_obstale = self.Avoidance_Strategy()
					if(_have_obstale == True):
						x = self.slow_vel_x
						z = z * 0.8 + _obstacle
					else:
						z += _obstacle
				self.Robot_Vel([x, z])
				self.prev_RPdis = RPdis
			else:
				x = -self.vel_x
				z = 0
				self.Robot_Vel([x, z])
				self.state = 2 if(self._front >= 0) else 3
		elif(self.state == 2):
			RPang = Norm_Angle(-80 - self._front)
			if(abs(RPang) > self.error_ang):
				z = self.find_ball_vel_z if(RPang > 0) else -self.find_ball_vel_z
				if(goal_size > goal_size_filter):
					self.state = 4
				self.Robot_Vel([0, z])
			else:
				self.Robot_Stop()
				self.find_goal_count += 1
				self.state = 3 if(self.find_goal_count < 2) else 5
		elif(self.state == 3):
			RPang = Norm_Angle((80)-self._front)
			if(abs(RPang) > self.error_ang):
				z = self.find_ball_vel_z if(RPang > 0) else -self.find_ball_vel_z
				if(goal_size > goal_size_filter):
					self.state = 4
				self.Robot_Vel([0, z])
			else:
				self.Robot_Stop()
				self.find_goal_count += 1
				self.state = 2 if(self.find_goal_count < 2) else 5
		elif(self.state == 4):
			self.find_goal_count = 0
			_,timer_flag = self.Timer_10.Process()
			if(goal_offset > 0 and abs(goal_offset) > goal_offset_error):
				z = -self.vel_z
				self.Robot_Vel([0,z])
			elif(goal_offset < 0 and abs(goal_offset) > goal_offset_error):
				z = self.vel_z
				self.Robot_Vel([0,z])
			if(timer_flag == True):#time out
				self.state = 5
				print("fuck")
			elif(abs(goal_offset) < goal_offset_error):
				self.state = 6
				self.Robot_Stop()
		elif(self.state == 5):
			self.find_goal_count = 0
			RPang = Norm_Angle(self.Get_RP_Angle(self.goal) - self._front)
			if(abs(RPang) > 0.5):
				z = self.z_speed_planning(RPang)
				self.Robot_Vel([0, z])
			else:
				self.Robot_Stop()
				z=0
				self.state = 6
		elif(self.state == 6):
			self.find_goal_count = 0
			time.sleep(0.5)
			self.Robot_Stop()
			time.sleep(0.2)
			self.pub_shoot.publish()
			self.state = 7
		elif(self.state == 7):
			self.Robot_Stop()
			print("finish")
			self.lostball = False
			self.Robot_Stop()
			self.state = 0

			self.behavior = FIND_BALL

			self.ballcolor = None
			self.balldis = 999
			self.ballang = 999
			self.ballarea = 0
			self.goal = self.findballpos
	def	Far_Shoot_Strategy(self):
		front_goal = copy.deepcopy(self.goal)
		if(self.state == 0):
			RPang = Norm_Angle(self.Get_RP_Angle(front_goal) - self._front)
			if(abs(RPang) > self.error_ang):
				z = self.z_speed_planning(RPang)
				self.Robot_Vel([0, z])
			else:
				self.Robot_Stop()
				self.state = 1
		elif(self.state == 1):
			self.Robot_Stop()
			time.sleep(0.2)
			self.pub_shoot.publish()
			self.state = 2
		elif(self.state == 2):
			self.Robot_Stop()
			print("finish")
			self.lostball = False
			self.Robot_Stop()
			self.state = 0

			if(self.web_set_up and len(self.web_axis) != 0 and len(self.web_ang) != 0):
				self.behavior =BEGINER
			else:
				self.behavior = FIND_BALL if(self._pos[0] > 1.5) else FIND_BALL2

			self.ballcolor = None
			self.balldis = 999
			self.ballang = 999
			self.ballarea = 0
			self.goal = self.findballpos
	def Process(self):
		self._behavior = self.behavior_to_strings(self.behavior)
		self._color = self.color_to_strings(self.ballcolor)
		self._double_point = self.color_to_strings(self._double)
		if(self._start):
			print(self._double_point, self._behavior, self.state, self._color, self.balldis, self.ballang)
			self.Robot_Moving()
			switcher = {
				INIT : self.Init_Strategy,
				BEGINER : self.Beginer_Strategy,
				WEB_GO : self.web_go_Strategy,
				FIND_BALL : self.Find_Ball_Strategy,
				CATCH_BALL : self.Catch_Ball_Strategy,
				GOAL : self.Goal_Strategy,
				FIND_BALL2 : self.Find_Ball_Strategy,
				SHORT_SHOOT : self.Short_Shoot_Strategy,
				FAR_SHOOT : self.Far_Shoot_Strategy
			}
			run_Process = switcher.get(self.behavior, self.Robot_Stop)
			run_Process()
		else:
			self.Robot_Stop()
			self.state = 0
			self.behavior = INIT

	def Process2(self):
		self._behavior = self.behavior_to_strings(self.behavior)
		self._color = self.color_to_strings(self.ballcolor)
		self._double_point = self.color_to_strings(self._double)
		if(self._start):
			print(self._behavior,self.state)
			self.Robot_Moving()
			switcher = {
				INIT : self.Init_Strategy,
				BEGINER :self.Beginer_Strategy,
				WEB_GO :self.web_go_Strategy,
				FIND_BALL : self.Find_Ball_Strategy3,
				CATCH_BALL : self.Catch_Ball_Strategy2,
				GOAL : self.Goal_Strategy2,
				SHORT_SHOOT : self.Short_Shoot_Strategy2

				# FIND_BALL2 : self.Find_Ball_Strategy3,
				# FIND_BALL2 : self.test,
				# GOAL : self.test,
			}
			run_Process = switcher.get(self.behavior,self.Robot_Stop)
			run_Process()
		else:
			self.Robot_Stop()
			self.state = 0
			self.behavior = INIT

	def test(self, sleep_time = 3):
		self.Robot_Stop()
		self.counter +=1
		if(self.counter / 100.0 == 1 or self.counter / 100.0 == 2 \
		or self.counter / 100.0 == 3 or self.counter / 100.0 == 4 \
		or self.counter / 100.0 == 5 or self.counter / 100.0 == 6):
			print(self.counter)
		if(self.counter == (sleep_time // 0.0333)):
			self.counter = 0
			self.behavior = WEB_GO

	def web_go_Strategy(self):
		if((len(self.web_axis) != 0 or len(self.web_ang) != 0) or self.state == 2):
			if(self.state == 0 or self.state == 2):
				RPang = self.Get_RP_Angle(self.goal) - self._front if(self.state == 0) else self.goalang - self._front
				RPang = Norm_Angle(RPang)
				if(abs(RPang) > self.error_ang):
					z = self.vel_z if(RPang > 0) else -self.vel_z
					self.Robot_Vel([0, z])
				else:
					self.Robot_Stop()
					self.state = 1 if(self.state == 0) else 2
					self.behavior = FIND_BALL if(self.state == 2) else self.behavior
			elif(self.state == 1):
				print(self.goal)
				RPdis = self.Get_RP_Dis(self.goal)
				RPang = Norm_Angle(self.Get_RP_Angle(self.goal) - self._front)
				if(RPdis > self.error_dis):
					if(self.prev_RPdis >= RPdis):
						x = self.x_speed_planning(RPdis)
						z = self.z_speed_planning(RPang) if(abs(RPang) > self.error_ang) else 0
					else:
						x = 0
						z = 0
						# if(RPdis < 0.3):# exceed
						# 	self.state = 0
					if(RPdis > self.avoid_error_dis): # Avoidance
						_obstacle,_have_obstale = self.Avoidance_Strategy()
						if(_have_obstale == True):
							x = self.slow_vel_x
							z = z * 0.8 + _obstacle
						else:
							z += _obstacle
					self.Robot_Vel([x, z])
					self.prev_RPdis = RPdis
				else:
					# self.goal = self.web_axis.pop(0) if(len(self.web_axis) != 0) else self.goal
					self.goalang = self.web_ang.pop(0) if(len(self.web_ang) != 0) else self.goalang
					self.prev_RPdis = 999
					self.state = 2
					self.Robot_Stop()
			else:
				self.Robot_Stop()
		else:
			self.web_set_up = False
			self.Robot_Stop()
			self.behavior = FIND_BALL

	def Find_Ball_Strategy3(self, angle = 45):
		if(self.state == 0):
			if(self.lostball == False):
				self.ballcolor = None
				self.balldis = 999
				self.ballang = 999
			self.Robot_Stop()
			self.state = 1 if(self._front >= self.web_ang) else 2
		elif(self.state == 1 or self.state == 2):
			RPang = -angle + self.goalang - self._front if(self.state == 1)else angle + self.goalang - self._front
			RPang = Norm_Angle(RPang)
			if(abs(RPang) > self.error_ang):
				z = self.find_ball_vel_z if(RPang > 0) else -self.find_ball_vel_z
				self.Robot_Vel([0, z])
				if(self.lostball == True):
					if(self._ballsColor == self.ballcolor and self._ballsDis < 250):
						self.ballcolor = self._ballsColor
						self.balldis = self._ballsDis
						self.ballang = self._front
						print('find ball')
						self.Robot_Stop()
						self.behavior = CATCH_BALL
						self.state = 0
						self.lostball = False
				else:
					if(self._ballsDis < self.balldis):
						self.ballcolor = self._ballsColor
						self.balldis = self._ballsDis
						self.ballang = self._front
					if(self._ballsColor == self._double and self._ballsDis < 280 and self.balldis > 200):
						self.ballcolor = self._ballsColor
						self.balldis = self._ballsDis
						self.ballang = self._front
						print("double!!!!!!!!")
						self.Robot_Stop()
						self.behavior = CATCH_BALL
						self.state = 0
						self.lostball = False
			else:
				self.Robot_Stop()
				self.lostball = False
				self.behavior = CATCH_BALL
				self.state = 0

	def	Short_Shoot_Strategy2(self):
		front_goal = copy.deepcopy(self.goal)
		front_goal[0] = 2.5
		goal_offset = self._goal[self.ballcolor * 4]
		goal_offset_error = 30
		goal_size = self._goal[self.ballcolor * 4 + 2]
		goal_size_filter = 3000
		if(self.state == 0):
			RPang = Norm_Angle(self.Get_RP_Angle(front_goal) - self._front)
			if(abs(RPang) > self.error_ang):
				z= self.find_ball_vel_z if(RPang > 0) else -self.find_ball_vel_z
				self.Robot_Vel([0, z])
			else:
				self.Robot_Stop()
				self.state = 1
		elif(self.state == 1):
			RPdis = self.Get_RP_Dis(front_goal)
			RPang = Norm_Angle(self.Get_RP_Angle(front_goal) - self._front)
			if(self._pos[0] > front_goal[0]):
				self.Robot_Stop()
				self.state = 2 if(self._front >= 0) else 3
			if(RPdis > self.error_dis):
				if(self.prev_RPdis >= RPdis):
					x = self.x_speed_planning(RPdis)
					z = self.z_speed_planning(RPang) if(abs(RPang) > self.error_ang) else 0
				else:
					x = 0
					z = 0
					if(RPdis < self.error_dis):
						self.state = 0
				if(RPdis > self.avoid_error_dis):
					_obstacle,_have_obstale = self.Avoidance_Strategy()
					if(_have_obstale == True):
						x = self.slow_vel_x
						z = z * 0.8 + _obstacle
					else:
						z += _obstacle
				self.Robot_Vel([x, z])
				self.prev_RPdis = RPdis
			else:
				x = -self.vel_x
				z = 0
				self.Robot_Vel([x, z])
				self.state = 2 if(self._front >= 0) else 3
		elif(self.state == 2 or self.state == 3):
			RPang = -80 - self._front if(self.state == 2) else 80 - self._front
			RPang = Norm_Angle(RPang)
			if(abs(RPang) > self.error_ang):
				z = self.find_ball_vel_z if(RPang > 0) else -self.find_ball_vel_z
				if(goal_size > goal_size_filter):
					self.state = 4
				self.Robot_Vel([0, z])
			else:
				self.Robot_Stop()
				self.find_goal_count += 1
				self.state = 3 if(self.find_goal_count < 2) else 5
		elif(self.state == 4):
			self.find_goal_count = 0
			timer_flag = self.Timer_10.Process()
			if(goal_offset > 0 and abs(goal_offset) > goal_offset_error):
				z = -self.vel_z
				self.Robot_Vel([0,z])
			elif(goal_offset < 0 and abs(goal_offset) > goal_offset_error):
				z = self.vel_z
				self.Robot_Vel([0,z])
			if(timer_flag == True):#time out
				self.state = 5
				print("fuck")
			elif(abs(goal_offset) < goal_offset_error):
				self.state = 6
				self.Robot_Stop()
		elif(self.state == 5):
			self.find_goal_count = 0
			RPang = Norm_Angle(self.Get_RP_Angle(self.goal) - self._front)
			if(abs(RPang) > 0.5):
				z = self.z_speed_planning(RPang)
				self.Robot_Vel([0, z])
			else:
				self.Robot_Stop()
				z=0
				self.state = 6
		elif(self.state == 6):
			self.find_goal_count = 0
			time.sleep(0.5)
			self.Robot_Stop()
			time.sleep(0.2)
			self.pub_shoot.publish()
			self.state = 7
		elif(self.state == 7):
			self.Robot_Stop()
			print("finish")
			self.lostball = False
			self.Robot_Stop()
			self.state = 0
			print("behavior",self.behavior,"web_set_up",self.web_set_up)
			self.behavior =WEB_GO if(self.web_set_up) else FIND_BALL
			print("behavior",self.behavior)
			self.ballcolor = None
			self.balldis = 999
			self.ballang = 999
			self.ballarea = 0
			print("shoot_axis",self.web_axis)
			self.goal = self.findballpos if(len(self.web_axis) == 0) else self.web_axis.pop(0)
			print("shoot",self.goal)

	def Catch_Ball_Strategy2(self):
		self.lostball = False
		if(self.state == 0):
			if(self.ballang == 999):
				self.Robot_Stop()
				self.behavior = FIND_BALL
				self.state = 0
				self.ballcolor = None
				self.balldis = 999
				self.ballang = 999
			else:
				RBang = Norm_Angle(self.ballang - self._front)
				if(abs(RBang) > 2):
					z = self.vel_z if(RBang > 0) else -self.vel_z
					self.Robot_Vel([0, z])
				else:
					self.Robot_Stop()
					self.state = 1
		elif(self.state == 1):
			color = self.ballcolor
			if(self._ballsColor == self.ballcolor):
				color = self._ballsColor
			if(color != None):
				RBang = self._ballsAng
				if(abs(RBang) > self.error_ang):
					z = self.vel_z if(RBang > self.error_ang) else -self.vel_z
					self.Robot_Vel([0, z])
				else:
					self.Robot_Stop()
					self.state = 2
			else:
				self.Robot_Stop()
				self.state = 0
				self.behavior = FIND_BALL
				self.ballcolor = None
				self.ballang = 999
				self.ballarea = 0
		elif(self.state == 2):
			color = None
			if(self.ballcolor != None):
				if(self._object[self.ballcolor * 4 + 1] < 999):
					color = self.ballcolor
			if(color != None):
				RPdis = self._object[self.ballcolor * 4 + 1]
				RBang = -self._object[self.ballcolor * 4]
				if(RPdis > self.catchBallDis):
					if(self.prev_RPdis >= RPdis):
						x = self.vel_x if(RPdis > 220) else self.slow_vel_x
						if(abs(RBang) > self.error_ang):
							z = self.vel_z if(RBang > 0) else -self.vel_z
						else:
							z = 0
					else:
						x = 0
						z = 0
						if(RPdis < 0.3):# exceed
							self.state = 1
					if(RPdis > 220):
						_obstacle,_have_obstale = self.Avoidance_Strategy()
						if(_have_obstale == True):
							x = self.slow_vel_x
						z += _obstacle
					self.Robot_Vel([x, z])
					self.prev_RPdis = RPdis
				else:
					x = 0.05
					self.Catch_Ball(1)
					switcher = {
						0 : GOAL,
						1 : SHORT_SHOOT,
						2 : FAR_SHOOT
					}
					self.behavior = switcher.get(self.strategy,GOAL)
					self.goal = self._goalarea[color]
					self.state = 0
					self.Robot_Stop()
			else:
				self.state = 0
				self.lostball = True
				print('lostball')
				self.behavior = FIND_BALL
				self.ballang = 999
				self.ballarea = 0
				self.Robot_Stop()

	def	Goal_Strategy2(self):
		front_goal = copy.deepcopy(self.goal)
		front_goal[0] = 2.5
		#=======================go to front of goal area===================
		if(self.state == 0):
			RPang = Norm_Angle(self.Get_RP_Angle(front_goal) - self._front)
			if(abs(RPang) > self.error_ang):
				z = self.find_ball_vel_z if(RPang > 0) else -self.find_ball_vel_z
				self.Robot_Vel([0, z])
			else:
				self.Robot_Stop()
				self.state = 1
		elif(self.state == 1):
			RPdis = self.Get_RP_Dis(front_goal)
			RPang = Norm_Angle(self.Get_RP_Angle(front_goal) - self._front)
			if(RPdis > self.error_dis):
				if(self.prev_RPdis >= RPdis):
					x = self.x_speed_planning(RPdis)
 					z = self.z_speed_planning(RPang) if(abs(RPang) > self.error_ang) else 0
				else:
					x = 0
					z = 0
					if(RPdis < self.error_dis):
						self.state = 0
				if(RPdis > self.avoid_error_dis):
					_obstacle,_have_obstale = self.Avoidance_Strategy()
					if(_have_obstale == True):
						x = self.slow_vel_x
					z += _obstacle
				self.Robot_Vel([x, z])
				self.prev_RPdis = RPdis
			else:
				x = -self.vel_x
				z = 0
				self.Robot_Vel([x, z])
				self.state = 2
		#=======================go to goal area===================
		elif(self.state == 2):
			RPang = Norm_Angle(self.Get_RP_Angle(self.goal) - self._front)
			if(abs(RPang) > self.goal_error_ang):
				z = self.z_speed_planning(RPang)
				self.Robot_Vel([0, z])
			else:
				self.Robot_Stop()
				z = 0
				time.sleep(1)
				self.state = 3
		elif(self.state == 3):
			RPdis = self.Get_RP_Dis(self.goal)
			RPang = Norm_Angle(self.Get_RP_Angle(self.goal) - self._front)
			print('self.goal',self.goal)
			print('RPdis',RPdis)
			if(RPdis > self.error_dis):
				if(self.prev_RPdis >= RPdis):
					x = self.x_speed_planning(RPdis)
					z = self.z_speed_planning(RPang) if(abs(RPang) > self.error_ang) else 0
				else:
					x = 0
					z = 0
					if(RPdis < self.error_dis):
						self.state = 0
				if(RPdis > self.avoid_error_dis):
					_obstacle,_have_obstale = self.Avoidance_Strategy()
					if(_have_obstale == True):
						x = self.slow_vel_x
					z += _obstacle

				self.Robot_Vel([x, z])
				self.prev_RPdis = RPdis
			else:
				self.Robot_Stop()
				self.Catch_Ball(0)
				self.state = 4
		elif(self.state == 4):
			RPdis = self.Get_RP_Dis([2.7,self.goal[1]])
			RPang = Norm_Angle(180 - (self.Get_RP_Angle([2.7,self.goal[1]]) - self._front))
			print('RPang',RPang)
			if(abs(RPang) > self.error_ang):
				z = -self.vel_z if(RPang > 0) else self.vel_z
				self.Robot_Vel([0, z])
			else:
				self.Robot_Stop()
				self.state = 5
		elif(self.state == 5):
			RPdis = self.Get_RP_Dis([2.7,self.goal[1]])
			RPang = Norm_Angle(180 - (self.Get_RP_Angle([2.7,self.goal[1]]) - self._front))
			if(RPdis > self.error_dis):
				if(self.prev_RPdis >= RPdis):
					x = -self.vel_x
					if(abs(RPang) > self.error_ang):
						z = -self.vel_z if(RPang > 0) else self.vel_z
					else:
						z = 0
				else:
					x = 0
					z = 0
					self.state = 4
				self.Robot_Vel([x, z])
				self.prev_RPdis = RPdis
			else:
				self.Robot_Stop()
				print("finish")
				self.lostball = False
				self.Robot_Stop()
				self.state = 0
				self.behavior = FIND_BALL
				self.ballcolor = None
				self.balldis = 999
				self.ballang = 999
				self.ballarea = 0
				self.goal = self.findballpos
		else:
			self.Robot_Stop()

def main():
	rospy.init_node('strategy', anonymous=True)
	strategy = Strategy()
	# 30 hz
	rate = rospy.Rate(30)
	while not rospy.is_shutdown():
		if(strategy._pos):
			# strategy.Process()
			strategy.Process2()
		rate.sleep()
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting down")

if __name__ == "__main__":
	main()
