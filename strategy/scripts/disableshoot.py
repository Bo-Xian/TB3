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

# rostopic msg
from std_msgs.msg import Int32
from std_msgs.msg import Int32,Bool
from std_msgs.msg import Int32,Empty
from std_msgs.msg import Int32MultiArray
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from vision.msg import aim,aim2


# Define Behavior
INIT = 0
BEGINER = 1
FIND_BALL = 2
CATCH_BALL = 3
GOAL = 4
FIND_BALL2 = 5

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

def zerolistmaker(n):
    listofzeros = [0] * n
    return listofzeros

class NodeHandle(object):
	def __init__(self):
		self._scan = None
		self._pos = None
		self._front = None

		self._start = 0

		self._ballsColor = None
		self._ballsDis = None
		self._ballsAng = None
		self._ballsArea = None
		self._ballsColor2 = []
		self._ballsCatch = None
		self._catchBallDis = 70
		self._scan = zerolistmaker(360)
		self._double = None

		if(SIMULATEION_FLAG):
			self.pub_vel = rospy.Publisher('robot1/cmd_vel',Twist, queue_size = 1)

			self.sub_scaninfo = rospy.Subscriber("robot1/scan",LaserScan,self.Set_ScanInfo)
			self.sub_odom = rospy.Subscriber("robot1/odom",Odometry,self.Set_Odom)
		else:
			self.pub_vel = rospy.Publisher('cmd_vel',Twist, queue_size = 1)
			self.pub_arm = rospy.Publisher('/tb3/arm',Int32, queue_size = 1)
			self.pub_shoot = rospy.Publisher("tb3/shoot",Empty, queue_size = 1)
			self.sub_scaninfo = rospy.Subscriber("scan",LaserScan,self.Set_ScanInfo)
			self.sub_odom = rospy.Subscriber("odom",Odometry,self.Set_Odom)
			self.sub_double = rospy.Subscriber("tb3/strategy/double",Int32,self.Set_Double)
		self.sub_balls = rospy.Subscriber("tb3/ball",Int32MultiArray,self.Set_Balls)
		#self.sub_ball2s = rospy.Subscriber("tb3/ball2",aim2,self.Set_Ball2s)
		self.sub_start = rospy.Subscriber("tb3/strategy/start",Int32,self.Set_Start)
		self.sub_save = rospy.Subscriber("tb3/save", Int32, self.Set_Param)
		self.pub_moving = rospy.Publisher("tb3/strategy/moving",Bool, queue_size = 1)
		if rospy.has_param("tb3/center"):
			self.catchBallDis_param = rospy.get_param("tb3/center")
			self.catchBallDis = self.catchBallDis_param[2]
			#print("catchBallDis", self.catchBallDis)
	print("ready")
	def Set_ScanInfo(self,msg):
		#self._scan = msg.ranges
		for i in range(len(msg.ranges)):
			if(msg.ranges[i] == 0.0):
				self._scan[i] = 999
			else:
				self._scan[i] = msg.ranges[i]
		#print(len(msg.ranges))
				#print(msg.ranges[i])
		#for i in range(len(self._scan)):
		#	if(self._scan[i] < 0.2):
		#		print(i)
		#print("scan", self._scan)
		#print("scan size", len(self._scan))
	def Set_Odom(self,msg):
		self._pos = [msg.pose.pose.position.x,msg.pose.pose.position.y]
		(r,p,y) = tf.transformations.euler_from_quaternion([msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w])

		self._front = Rad2Deg(y)
	def Set_Start(self,msg):
		self._start = msg.data
		if(not self._start):
			print('stop')
			self.Robot_Stop()
		else:
			print('start')

	def Set_Balls(self,msg):
		self._ballsColor = msg.data[0]
		self._ballsAng = msg.data[1]
		self._ballsDis = msg.data[2]
		#self._ballsArea = msg.area
	def Set_Double(self,msg):
		self._double = msg.data
		print(self.color_to_strings(self._double))
	def Set_Ball2s(self,msg):
		self._ballsColor2 = msg.color2
		self._ballsCatch = msg.extbot2
	def Set_Param(self, msg):
		self.catchBallDis_param = rospy.get_param("tb3/center")
		self.catchBallDis = self.catchBallDis_param[2]
		#print("catchBallDis", self.catchBallDis)

class Strategy(NodeHandle):
	def __init__(self):
		super(Strategy,self).__init__()
		self.behavior = INIT
		self.initpos = [[0.0,0.0]]
		self.findballpos = [1.9,0.05]
		#self.findballpos = [0.4,0.0]
		self.goal = None
		self._goalarea = {0:[3.1,-0.6],1:[3.1,0.0],2:[3.1,0.3],3:[3.1,0.6],4:[3.1,-0.3]}
		#self._goalarea = {'red':[2.7,-0.6],'blue':[2.7,0.0],'yellow':[2.7,0.3],'white':[2.7,0.6],'black':[2.7,-0.3]}
		#self._goalarea = {'red':[0.5,-0.5],'blue':[0.5,0.0],'yellow':[0.5,0.25],'white':[0.5,0.5],'black':[0.5,-0.25]}
		self.prev_RPdis = 999

		self.state = 0

		# self.vel_x = 0.8
		self.vel_x = 0.3
		self.vel_z = 0.2
		self.error_ang = 1.0
		self.error_dis = 0.025
		self.error_area = 28000
		self.error_catchArea = 80000
		self.findang = 30
		self.error_ballang = 5.0

		self.ballcolor = None
		self.balldis = 999
		self.ballang = 999
		self.ballarea = 0

		self.lostball = False
	def behavior_to_strings(self,argument):
		switcher = {
			0: "INIT",
			1: "BEGINER",
			2: "FIND_BALL",
			3: "CATCH_BALL",
			4: "GOAL",
			5: "FIND_BALL2"
		}
		return switcher.get(argument, "nothing")

	def color_to_strings(self,argument):
		switcher = {
			0: "Red",
			1: "Blue",
			2: "Yellow",
			3: "White",
			4: "Black",
			999:"Error",
		}
		return switcher.get(argument, "None")

	def Process(self):
		self._behavior = self.behavior_to_strings(self.behavior)
		self._color = self.color_to_strings(self.ballcolor)
		self._double_point = self.color_to_strings(self._double)
		if(self._start):
			print(self._double_point,self._behavior,self.state,self._color,self.balldis,self.ballang)
			self.Robot_Moving()
			if(self.behavior == INIT):

				self.Init_Strategy()
				#print(self.goal)
			elif(self.behavior == BEGINER):
				self.Beginer_Strategy()
			elif(self.behavior == FIND_BALL):
				self.Find_Ball_Strategy()
			elif(self.behavior == CATCH_BALL):
				self.Catch_Ball_Strategy()
			elif(self.behavior == GOAL):
				self.Goal_Strategy()
			elif(self.behavior == FIND_BALL2):
				self.Find_Ball_Strategy2()
			else:
				self.Robot_Stop()
		else:
			#self.Avoidance_Strategy()
			self.Robot_Stop()
			self.behavior = INIT
			self.state = 0

	def Robot_Vel(self,vel):
		twist = Twist()
		twist.linear.x = vel[0]
		twist.angular.z = vel[1]

		self.pub_vel.publish(twist)

	def Catch_Ball(self,control):
		time.sleep(0.5)
		self.pub_arm.publish(control)

	def Robot_Stop(self):
		twist = Twist()
		twist.linear.x = 0
		twist.angular.z = 0

		self.pub_vel.publish(twist)

	def Robot_Moving(self):
		self.pub_moving.publish(True)
    	# robot to pose angle

	def Get_RP_Angle(self,pos):
		return Rad2Deg(math.atan2((pos[1]-self._pos[1]),(pos[0]-self._pos[0])))

	# robot to pose distance
	def Get_RP_Dis(self,pos):
		return math.sqrt(pow(pos[0]-self._pos[0],2.0)+pow(pos[1]-self._pos[1],2.0))

#===============================================
	def x_speed_planning(self,distance):
		max_distance = 1.5
		min_distance = 0.4
		max_speed = 0.6
		min_speed = 0.2
		planning_speed = self.vel_x
		planning_speed = (abs(distance)-min_distance)/(max_distance-min_distance)*(max_speed-min_speed)+min_speed
		if(abs(distance) > max_distance):
			planning_speed = max_speed
		elif(abs(distance) < min_distance):
			planning_speed = min_speed
		#A-Amin/Amax-Amin=B-Bmin/Bmax-Bmin
		#print("x: ", planning_speed)
		return planning_speed
#===============================================
	def z_speed_planning(self,angle):
		max_angle = 30
		min_angle = 10
		max_speed = 0.6
		min_speed = 0.2
		planning_speed = self.vel_x
		planning_speed = (abs(angle)-min_angle)/(max_angle-min_angle)*(max_speed-min_speed)+min_speed
		if(abs(angle) > max_angle):
			planning_speed = max_speed
		elif(abs(angle) < min_angle):
			planning_speed = min_speed
		#A-Amin/Amax-Amin=B-Bmin/Bmax-Bmin
		#print("z: ",angle, planning_speed)
		return planning_speed
#===============================================
	def Avoidance_Strategy(self):
		obstacle = 0
		obstacle_num = 0
		avoidance_distance = 0.5
		avoidance_angle = 60
		avoidance_vel = 0.5
		have_obstale = False

		for i in range(len(self._scan)-10):
			if(i>len(self._scan)):
				break
			if(self._scan[i]<0.35):
				have_obstale = True
			if(self._scan[i] < avoidance_distance):
				#print(i)
				if(i < avoidance_angle and i > 0):
					#left side
					obstacle_num +=1
					obstacle -= (avoidance_distance-self._scan[i])
				if(i > (360-avoidance_angle) or i < 0):
					#right side
					obstacle_num +=1
					obstacle += (avoidance_distance-self._scan[i])
		if(obstacle_num):
			obstacle = obstacle*10/obstacle_num
			if(obstacle > avoidance_vel):
				obstacle = avoidance_vel
			elif(obstacle < avoidance_vel*-1):
				obstacle = avoidance_vel*-1
			#print (obstacle)
		return obstacle, have_obstale
	# strategy
#==========================================
	def Init_Strategy(self):
		#self.initpos = [[0.0,0.0]]
		self.initpos = [[0.1,0.0]]
		#self.initpos = [[2.2,0.0]]
		#self.initpos = [[0.0,-0.3],[0.3,-0.3],[0.3,0.0]]
		#self.initpos = [[2.7,0.0],[1.5,0.0],[1.5,-0.35],[2.7,-0.35],[1.5,-0.35],[1.5,-0.66],[2.7,-0.66],[1.5,-0.66],[1.5,0.35],[2.7,0.35],[1.5,0.35],[1.5,0.66],[2.7,0.66]]
		#self.initpos = [[2.7,0.0],[1.5,0.0],[1.5,-0.32],[2.7,-0.32],[1.5,-0.32],[1.5,-0.66],[2.7,-0.66],[1.5,-0.66],[1.5,0.32],[2.7,0.32],[1.5,0.32],[1.5,0.66],[2.7,0.66]]

		self.findballpos = [1.9,0.05]
		#self.findballpos = [0.4,0.0]
		self.goal = None
		#0red 1blue 2yellow
		#self._goalarea = {0:[2.7,-0.6],1:[2.7,0.0],2:[2.7,0.3],3:[2.7,0.6],4:[2.7,-0.3]}
		self._goalarea = {0:[3.2,0.0],1:[3.2,-0.5],2:[3.2,0.5]}
		#0red 1blue 2yellow

		#self._goalarea = {'red':[2.7,-0.6],'blue':[2.7,0.0],'yellow':[2.7,0.3],'white':[2.7,0.6],'black':[2.7,-0.3]}
		#self._goalarea = {'red':[0.5,-0.5],'blue':[0.5,0.0],'yellow':[0.5,0.25],'white':[0.5,0.5],'black':[0.5,-0.25]}
		self.prev_RPdis = 999

		self.state = 0

		# self.vel_x = 0.8
		self.vel_x = 0.4
		self.vel_z = 0.3
		self.find_ball_vel_z = 0.6
		#self.error_ang = 1.0
		self.error_ang = 3.0
		self.error_dis = 0.025
		self.error_area = 28000
		self.error_catchArea = 80000
		self.findang = 30
		self.error_ballang = 5.0

		self.ballcolor = None
		self.balldis = 999
		self.ballang = 999
		self.ballarea = 0

		self.Catch_Ball(0)
		self.goal = self.initpos.pop(0)
		self.behavior = BEGINER
#========================================
	def Beginer_Strategy(self):

		if(self.state == 0):
			RPang = Norm_Angle(self.Get_RP_Angle(self.goal)-self._front)

			if(abs(RPang) > 2):
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
					 		x = self.x_speed_planning(RPdis)
							z = self.z_speed_planning(RPang)
						else:
							x = self.x_speed_planning(RPdis)
							z = -self.z_speed_planning(RPang)
					else:
						x = self.x_speed_planning(RPdis)
						z = 0
				else:
					x = 0
					z = 0
					if(RPdis<0.3):
						self.state = 0
				if(RPdis>0.3):
					a,b = self.Avoidance_Strategy()
					z+=a
					if(abs(z)>0.2 or b==True):
						x=0.1
				self.Robot_Vel([x,z])
				self.prev_RPdis = RPdis
			else:
				if(len(self.initpos) == 0):
					#self.behavior = FIND_BALL
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
			#self.balldis = 999
			if(self.lostball==False):
				self.ballcolor = None
				self.balldis = 999
				self.ballang = 999
				#print("recerch")
			if(self._front >= 0):
				RPang = Norm_Angle((90+self.findang)-self._front)
			else:
				RPang = Norm_Angle(-(90+self.findang)-self._front)
			if(abs(RPang) > self.error_ang):
				if(RPang > 0):
					x = 0
					z = self.find_ball_vel_z
				else:
					x = 0
					z = -self.find_ball_vel_z
				self.Robot_Vel([x,z])
			else:
				self.Robot_Stop()
				if(self._front >= 0):
					self.state = 1
				else:
					self.state = 2
		elif(self.state == 1):
			RPang = Norm_Angle(-(90+self.findang)-self._front)
			if(abs(RPang) > self.error_ang):
				if(RPang > 0):
					x = 0
					z = self.find_ball_vel_z
				else:
					x = 0
					z = -self.find_ball_vel_z
				self.Robot_Vel([x,z])
				if(self.lostball==True):
					if(self._ballsColor==self.ballcolor and self._ballsDis<250):
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
					if(self._ballsColor==self._double and self._ballsDis<280 and self.balldis>200):
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
		elif(self.state == 2):
			RPang = Norm_Angle(90+self.findang-self._front)
			if(abs(RPang) > self.error_ang):
				if(RPang > 0):
					x = 0
					z = self.find_ball_vel_z
				else:
					x = 0
					z = -self.find_ball_vel_z
				self.Robot_Vel([x,z])
				if(self.lostball==True):
					if(self._ballsColor==self.ballcolor and self._ballsDis<250):
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
					if(self._ballsColor==self._double and self._ballsDis<280 and self.balldis>200):
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
			#self.balldis = 999
			if(self.lostball==False):
				self.ballcolor = None
				self.balldis = 999
				self.ballang = 999
			if(self._front >= 0):
				RPang = Norm_Angle((self.findang)-self._front)
			else:
				RPang = Norm_Angle(-(self.findang)-self._front)
			if(abs(RPang) > self.error_ang):
				if(RPang > 0):
					x = 0
					z = self.find_ball_vel_z
				else:
					x = 0
					z = -self.find_ball_vel_z
				self.Robot_Vel([x,z])
			else:
				self.Robot_Stop()
				if(self._front >= 0):
					self.state = 1
				else:
					self.state = 2
		elif(self.state == 1):
			RPang = Norm_Angle(-(self.findang)-self._front)
			if(abs(RPang) > self.error_ang):
				if(RPang > 0):
					x = 0
					z = self.find_ball_vel_z
				else:
					x = 0
					z = -self.find_ball_vel_z
				self.Robot_Vel([x,z])

				if(self.lostball==True):
					if(self._ballsColor==self.ballcolor and self._ballsDis<250):
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
					if(self._ballsColor==self._double and self._ballsDis<280 and self.balldis>200):
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
		elif(self.state == 2):
			RPang = Norm_Angle(self.findang-self._front)
			if(abs(RPang) > self.error_ang):
				if(RPang > 0):
					x = 0
					z = self.vel_z
				else:
					x = 0
					z = -self.vel_z
				self.Robot_Vel([x,z])
				if(self.lostball==True):
					if(self._ballsColor==self.ballcolor and self._ballsDis<250):
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
					if(self._ballsColor==self._double and self._ballsDis<280 and self.balldis>200):
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
				if(self._pos[0]>1.0):
					self.behavior = FIND_BALL
				else:
					self.behavior = FIND_BALL2
				self.state = 0
				self.ballcolor = None
				self.balldis = 999
				self.ballang = 999
			else:
				RBang = Norm_Angle(self.ballang-self._front)
				if(abs(RBang) > 2):
					if(RBang > 0):
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
			#color = None
			color = self.ballcolor
			#for i in range(len(self._ballsColor)):
			if(self._ballsColor == self.ballcolor):
				color = self._ballsColor
				#break
			if(color!=None):
				RBang = self._ballsAng
				if(abs(RBang) > self.error_ang):
					if(RBang > self.error_ang):
						x = 0
						z = self.vel_z
					else:
						x = 0
						z = -self.vel_z
					self.Robot_Vel([x,z])
				else:
					self.Robot_Stop()
					self.state = 2
			else:
				self.Robot_Stop()
				self.state = 0
				if(self._pos[0]>1.0):
					self.behavior = FIND_BALL
				else:
					self.behavior = FIND_BALL2
				self.ballcolor = None
				self.ballang = 999
				self.ballarea = 0
		elif(self.state == 2):
			color = None
			if(self._ballsColor == self.ballcolor):
					color = self._ballsColor
			if(color!=None):
				#print(color)
				RPdis = self._ballsDis
				RBang = self._ballsAng
				#if(RPdis < self.error_area):
				if(RPdis > self.catchBallDis):
					if(self.prev_RPdis >= RPdis):
						#if(RPdis > 220):
						if(RPdis > 100):
							if(abs(RBang) > self.error_ang):
								if(RBang > 0):
									x = self.vel_x
									z = self.vel_z
								else:
									x = self.vel_x
									z = -self.vel_z
							else:
								x = self.vel_x
								z = 0
						else:
							if(abs(RBang) > self.error_ang):
								if(RBang > 0):
									x = self.vel_x-self.vel_z
									z = self.vel_z
								else:
									x = self.vel_x-self.vel_z
									z = -self.vel_z
							else:
								x = self.vel_x-self.vel_z
								z = 0
					else:
						x = 0
						z = 0
						#self.state = 1
						if(RPdis<0.3):
							self.state = 1
					if(RPdis>0.3):
						a,b = self.Avoidance_Strategy()
						z+=a
						if(abs(z)>0.2 or b==True):
							x=0.1
					self.Robot_Vel([x,z])
					self.prev_RPdis = RPdis
				else:
					self.Catch_Ball(1)
					self.behavior = GOAL
					self.goal = self._goalarea[color]
					self.state = 0
					self.Robot_Stop()
			else:
				self.state = 0
				self.lostball=True
				print('lostball')
				if(self._pos[0]>1.0):
					self.behavior = FIND_BALL
				else:
					self.behavior = FIND_BALL2
				self.ballang = 999
				self.ballarea = 0
				#self.ballcolor = None
				self.Robot_Stop()

	def	Goal_Strategy(self):
		front_goal = copy.deepcopy(self.goal)
		front_goal[0] = 2.5
		#=======================go to front of goal area===================
		if(self.state == 0):
			RPang = Norm_Angle(self.Get_RP_Angle(front_goal)-self._front)
			RBang = 0.0
			if(abs(RPang) > 1):
				if(RPang > 0):
						x = 0
						z = self.find_ball_vel_z
				else:
						x = 0
						z = -self.find_ball_vel_z
				self.Robot_Vel([x,z])
			else:
				self.Robot_Stop()
				self.state = 1
		elif(self.state == 1):
			RPdis = self.Get_RP_Dis(front_goal)
			RPang = Norm_Angle(self.Get_RP_Angle(front_goal)-self._front)
			if(RPdis > self.error_dis):
				if(self.prev_RPdis >= RPdis):
					if(abs(RPang) > self.error_ang):
						if(RPang > 0):
					 		x = self.x_speed_planning(RPdis)
							z = self.z_speed_planning(RPang)
						else:
							x = self.x_speed_planning(RPdis)
							z = -self.z_speed_planning(RPang)
					else:
						x = self.x_speed_planning(RPdis)
						z = 0
				else:
					x = 0
					z = 0
					#self.state = 0
				#self.Robot_Vel([x,z])
					if(RPdis<0.3):
						self.state = 0
				if(RPdis>0.3):
					a,b = self.Avoidance_Strategy()
					z+=a
					if(abs(z)>0.2 or b==True):
						x=0.1
				self.Robot_Vel([x,z])
				self.prev_RPdis = RPdis
			else:
				x = -self.vel_x
				z = 0
				self.Robot_Vel([x,z])
				self.state = 2



		#=======================go to goal area===================
		elif(self.state == 2):
			RPang = Norm_Angle(self.Get_RP_Angle(self.goal)-self._front)
			#RBang = 0.0
			temp_speed = 0
			if(abs(RPang)>10):
				temp_speed = 0.4
			else:
				temp_speed = 0.1
			if(abs(RPang) > 0.5):

				if(RPang > 0):
						x = 0
						z = temp_speed
				else:
						x = 0
						z = -temp_speed
				self.Robot_Vel([x,z])
			else:
				self.Robot_Stop()
				x=0
				z=0
				time.sleep(1);
				self.state = 3
		'''
		#================shoot strategy===========================
		elif(self.state == 3):
				self.pub_shoot.publish();
				time.sleep(1);
				self.Robot_Stop()
				self.state = 4
		elif(self.state == 4):
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
		'''
		elif(self.state == 3):
			RPdis = self.Get_RP_Dis(self.goal)
			RPang = Norm_Angle(self.Get_RP_Angle(self.goal)-self._front)
			if(RPdis > self.error_dis):
				if(self.prev_RPdis >= RPdis):
					if(abs(RPang) > self.error_ang):
						if(RPang > 0):
					 		x = self.x_speed_planning(RPdis)
							z = self.z_speed_planning(RPang)
						else:
							x = self.x_speed_planning(RPdis)
							z = -self.z_speed_planning(RPang)
					else:
						x = self.x_speed_planning(RPdis)
						z = 0
				else:
					x = 0
					z = 0
					#self.state =2
				#self.Robot_Vel([x,z])
					if(RPdis<0.3):
						self.state = 0
				if(RPdis>0.3):
					z+=self.Avoidance_Strategy()
					if(abs(z)>0.2):
						x=0.1
				self.Robot_Vel([x,z])
				self.prev_RPdis = RPdis
			else:
				self.Robot_Stop()
				self.Catch_Ball(0)
				self.state = 4
		#========================back=================================
		elif(self.state == 4):

			RPdis = self.Get_RP_Dis([2.7,self.goal[1]])
			RPang = Norm_Angle(180-(self.Get_RP_Angle([2.7,self.goal[1]])-self._front))
			print('rpang',RPang)
			if(abs(RPang) > self.error_ang):
				if(RPang > 0):
						x = 0
						z = -self.vel_z
				else:
						x = 0
						z = self.vel_z
				self.Robot_Vel([x,z])
			else:
				self.Robot_Stop()
				self.state = 5
		elif(self.state == 5):
			RPdis = self.Get_RP_Dis([2.7,self.goal[1]])
			RPang = Norm_Angle(180-(self.Get_RP_Angle([2.7,self.goal[1]])-self._front))
			if(RPdis > self.error_dis):
				if(self.prev_RPdis >= RPdis):
					if(abs(RPang) > self.error_ang):
						if(RPang > 0):
					 		x = -(self.vel_x-self.vel_z)
							z = -self.vel_z
						else:
							x = -(self.vel_x-self.vel_z)
							z = self.vel_z
					else:
						x = -(self.vel_x-self.vel_z)
						z = 0
				else:
					x = 0
					z = 0
					self.state =4
				self.Robot_Vel([x,z])
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
			#print("state ", self.state)
			#self.state = 0
			#self.behavior = FIND_BALL

def main():
	rospy.init_node('strategy', anonymous=True)
	strategy = Strategy()

	# 30 hz
	rate = rospy.Rate(30)
	i = 0
	while not rospy.is_shutdown():
		#if(strategy._scan):
            #for i in range(len(strategy._scan)):
            #    print(str(i)+"  "+str(strategy._scan[i]))
            #print("0 : " + str(strategy._scan[0]))
            #print("90 : " + str(strategy._scan[90]))
            #print("180 : " + str(strategy._scan[180]))
            #print("270 : " + str(strategy._scan[270]))
		if(strategy._pos):
			strategy.Process()
		rate.sleep()
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting down")

if __name__ == "__main__":
	main()
