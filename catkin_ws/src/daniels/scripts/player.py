#!/usr/bin/env python

import rospy
import os
import time

from jblink.jblink import JanasBlinkstick

from KIs.base.KiBase import KiBase

from std_msgs.msg import String
from geometry_msgs.msg import Twist, PoseStamped, PoseWithCovarianceStamped

from daniels.msg import SoccerPlayerSetup
from daniels.srv import *

import std_srvs.srv
from std_srvs.srv import *

from os.path import dirname, basename, isfile
import glob

class Player:

	active = False
	usedKI = None
	setup = SoccerPlayerSetup()
	bstick = JanasBlinkstick()
	pub = None
	ctrlSub = None

	def __init__(self):
		global rns
		rospy.init_node(rns,anonymous=False)
		self.bstick.blink("red",100)
		print "init Services..."
		self.initServices()
		time.sleep(1)
		self.bstick.blink("yellow",100)
		print "init Pubs'n'Subs"
		self.initPubsSubs()
		time.sleep(1)
		self.bstick.blink("green",100)
		print "Broadcasting: I want to play soccer !"
		self.iwanttoplay()
		time.sleep(1)

	def printInfo(self,info):
		if not self.pub is None:
			self.pub.publish(info)

	def initServices(self):
		self.setupService = rospy.Service('PlayerSetup',playerSetup,self.setupServiceCalled)
		self.startService = rospy.Service('Start',Trigger,self.startServiceCalled)
		self.stopService = rospy.Service('Stop',Trigger,self.stopServiceCalled)
		self.resetService = rospy.Service('Reset',Trigger,self.resetServiceCalled)
		self.kiService = rospy.Service('KIList',playerKIs,self.kiListServiceCalled)

	def kiListServiceCalled(self,req):
		result = playerKIsResponse()
		result.kis = self.getKIs(True)
		return result

	def initPubsSubs(self):
		self.ctrlPub = rospy.Publisher('navigation_velocity_smoother/raw_cmd_vel',Twist,queue_size=10)
		self.scanBall = rospy.Subscriber('DetectedBall',PoseStamped,self.gotABallDetection)
		self.amclPose = rospy.Subscriber('amcl_pose',PoseWithCovarianceStamped,self.amclPoseRecieved)

	def amclPoseRecieved(self, p):
		if not self.usedKI is None:
			self.usedKI.ownPositionUpdated(p.pose.pose)
		return

	@staticmethod
	def getTeamColor(teamnr):
		if teamnr==0:
			return "blue"
		if teamnr==1:
			return "yellow"
		return "red"

	def gotABallDetection(self,ballPoseStamped):
		if not self.usedKI is None and self.active and not self.setup.human:
			self.usedKI.ballDetected(ballPoseStamped)

	def resetServiceCalled(self,req):
		self.active = False
		self.printInfo('Gehe auf Startaufstellung')
		resp = TriggerResponse()
		resp.success = True
		resp.message = "Gehe zurueck auf Startaufstellung"
		if not self.usedKI is None:
			self.active = False
			self.usedKI.stop()
			self.usedKI.backToStart()
		return resp

	def stopServiceCalled(self,req):
		self.active = False
		self.printInfo('Roboter gestoppt')
		resp = TriggerResponse()
		resp.success = True
		resp.message = "Stoppe Roboter"
		if not self.usedKI is None:
			self.usedKI.stop()
		return resp

	def startServiceCalled(self,req):
		self.active = True
		self.printInfo('Roboter gestartet')
		resp = TriggerResponse()
		resp.success = True
		resp.message = "Starte Roboter"
		return resp

	def setupServiceCalled(self,req):
		self.printInfo('Setupdaten erhalten')
		if self.usedKI is None and req.sps.funktion>-1:
			self.usedKI = self.getKIs(False)[req.sps.funktion]()
			print self.usedKI
		self.setup = req.sps
		resp = playerSetupResponse()
		resp.sps = self.setup
		if not self.ctrlSub is None:
			self.ctrlSub.unregister()
		c = Player.getTeamColor(self.setup.team)
		self.ctrlSub = rospy.Subscriber('/HumanTwistMsg'+str(c),Twist,self.humanControlMsg)
		if not self.usedKI is None:
			self.usedKI.setupUpdated(self.setup)
		return resp
	
	def getKIs(self,onlyname):
		l = list()
		for ki in KiBase.__subclasses__():
			if onlyname:
				l.append(ki.__name__)
			else:
				l.append(ki)
		return l	

	#laedt alle KI-Module
	def findKIs(self):
		#alle Dateien suchen
		modules = glob.glob(dirname(__file__)+"/KIs/*.py")
		for f in modules:
			if isfile(f):
				if os.path.basename(f) != "__init__.py":
					class_name = os.path.basename(f)
					#Der Klassenname MUSS genau wie die Datei lauten
					class_name = os.path.splitext(class_name)[0]
					# Nun wird das Modul importiert
					module = __import__("KIs."+class_name)
					# und aus diesem Modul die entsprechende Klasse
					new_class = getattr(module,class_name)
		self.printInfo('KIs:')
		# Alle von KiBase vererbten Klassen auflisten
		for sc in KiBase.__subclasses__():
			self.printInfo(sc.__name__)

	def iwanttoplay(self):
		self.pub = rospy.Publisher('IWantToPlaySoccer', String,queue_size=1)
		self.findKIs()
		r = rospy.Rate(1)
		while not rospy.is_shutdown():
			self.updateKI()
			r.sleep()

	def updateKI(self):
		if self.active:
			if not self.usedKI is None and not self.setup.human:
				self.bstick.glow(Player.getTeamColor(self.setup.team))
				self.usedKI.update(1000)
				print("KI-ing...")
			else:
				self.bstick.blink(Player.getTeamColor(self.setup.team))
		else:
			self.bstick.glow("red")

	def humanControlMsg(self,move):
		if self.setup.human and self.active:
			self.ctrlPub.publish(move)
			

if __name__ == '__main__':
	rns = os.environ['ROS_NAMESPACE']
	if rns != 'None':
		player = Player()
	else:
		print "ROS_NAMESPACE nicht gesetzt!"
