#!/usr/bin/env python
# -*- coding: utf-8 -*-+

print("1")

a = []

if(len(a)==0):
	print("0")




# class test(object):
#     def __init__(self):
#         self.test_num = [0,1,2,3]
#         self.Q = None

#     def print_num(self):
#         self.Q = self.test_num
#         print(self.Q)

# def Norm_Angle(angle):
# 	if(angle > 180):
# 		angle -= 360
# 	elif(angle < -180):
# 		angle += 360
# 	return angle
# print(Norm_Angle(259),Norm_Angle(439))

# switcher ={
#     0 : "one",
#     1 : "two",
# }
# go =switcher.get(2,"NONO")
# print(go)
# a =101

# print(a/100.0)
# if(a/100.0 != 1):
#     print("QQ")


	# def Beginer_Strategy2(self):
	# 	if(self.state == 0 or self.state == 2):
	# 		RPang = self.goalang - self._front if(self.state == 2) else self.Get_RP_Angle(self.goal) - self._front
	# 		RPang = Norm_Angle(RPang)
	# 		print("RPang",RPang,"_front",self._front)
	# 		if(abs(RPang) > self.error_ang):
	# 			z = self.vel_z if(RPang > 0) else -self.vel_z
	# 			self.Robot_Vel([0, z])
	# 		elif(self.state == 0):
	# 			self.Robot_Stop()
	# 			self.state = 1
	# 		elif(self.state == 2):
	# 			self.Robot_Stop()
	# 			self.state = 0
	# 			self.behavior = FIND_BALL2
	# 		if(len(self.web_axis) == 0 and len(self.web_ang) == 0):
	# 			self.web_set_up = False
	# 			self.initpos.pop(0)
	# 	elif(self.state == 1):
	# 		RPdis = self.Get_RP_Dis(self.goal)
	# 		RPang = Norm_Angle(self.Get_RP_Angle(self.goal) - self._front)
	# 		print("RPdis",RPdis,"RPang",RPang)
	# 		if(RPdis > self.error_dis):
	# 			if(self.prev_RPdis >= RPdis):
	# 				x = self.x_speed_planning(RPdis)
	# 				z = self.z_speed_planning(RPang) if(abs(RPang) > self.error_ang) else 0
	# 			else:
	# 				x = 0
	# 				z = 0
	# 				if(RPdis < 0.3):
	# 					self.state = 2 if(self.web_set_up) else 0
	# 					print("state",self.state)
	# 			if(RPdis > self.avoid_error_dis): # Avoidance
	# 				_obstacle,_have_obstale = self.Avoidance_Strategy()
	# 				if(_have_obstale == True):
	# 					x = self.slow_vel_x
	# 					z = z * 0.8 + _obstacle
	# 				else:
	# 					z += _obstacle
	# 			print("x",x,"z",z)
	# 			self.Robot_Vel([x, z])
	# 			self.prev_RPdis = RPdis
	# 		else:
	# 			if((len(self.initpos) == 0 and not(self.web_set_up)) or \
	# 				(len(self.web_axis) == 0 and len(self.web_ang) == 0)):
	# 				self.behavior = FIND_BALL2
	# 			else:
	# 				if(self.web_set_up and len(self.web_axis) != 0 or \
	# 				not(self.web_set_up) and len(self.initpos) != 0):
	# 					self.goal = self.web_axis.pop(0) if(self.web_set_up) else self.initpos.pop(0)
	# 				self.goalang = self.web_ang.pop(0) if(self.web_set_up) else None
	# 				self.prev_RPdis = 999
	# 			self.state = 2 if(self.web_set_up) else 0
	# 			self.Robot_Stop()
	# 			self.counter = 0
	# 	else:
	# 		self.Robot_Stop()
