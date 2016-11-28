import rospy
import actionlib
import tf2_ros
import tf2_geometry_msgs
import os
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import time
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, Quaternion, PoseWithCovarianceStamped, PointStamped, Twist, PoseStamped, Point, Pose
from daniels.msg import SoccerPlayerSetup
import math
from GoalStates import GoalState


class KiBase(object):

	myPose = Pose()

	lastBallTime = None
	lastBallPose = Pose()
	lastBallPoseDiff = -1

	setup = SoccerPlayerSetup()
	center = Point()

	turnPub = None
	actionClient = None

	goalState = GoalState.none

	#tf_kram
	tfBuffer = None
	tfListener = None

	namespace = ""

	def __init__(self):
		self.namespace = os.environ['ROS_NAMESPACE']
		self.tfBuffer = tf2_ros.Buffer()
		self.tfListener = tf2_ros.TransformListener(self.tfBuffer)
		self.myPose = Pose()
		self.lastBallTime = rospy.Time(10)
		self.goal_sent = False
		rospy.on_shutdown(self.stop)
		self.turnPub = rospy.Publisher('mobile_base/commands/velocity',Twist,queue_size=10)
		self.actionClient = actionlib.SimpleActionClient('move_base', MoveBaseAction)
		rospy.loginfo("Basis-KI gestartet...")

	def getBallPose(self):
		return self.lastBallPose

	def ballDetected(self, ballPoseStamped):
		self.lastBallTime = rospy.Time.now()
		self.lastBallPoseDiff = self.distance(self.lastBallPose.position,ballPoseStamped.pose.position)
		self.lastBallPose = ballPoseStamped.pose
		return

	def timeSinceLastBall(self):
		n = int(rospy.Time.now().secs)
		l = int(self.lastBallTime.secs)
		return n - l

	def ownPositionUpdated(self, poseStamped):
		self.myPose=poseStamped

	def setupUpdated(self, setup):
		self.setup=setup
		self.center.x = (setup.zielTor.x+setup.eigenesTor.x)/2
		self.center.y = (setup.zielTor.y+setup.eigenesTor.y)/2
		return

	def update(self,time):
		return

	def gotoPose(self, posPose):
		print("Gehe zu Position")
		goal = MoveBaseGoal()
		goal.target_pose.header.frame_id = "map"
		goal.target_pose.header.stamp = rospy.Time.now()
		goal.target_pose.pose = posPose
		self.actionClient.send_goal(goal,done_cb = self.goalFeedback)
		return

	def getMeterFromBot(self):
		left = self.isBallOnLeftHalf()
		pose = Pose()
		pose.position.x = 0
		if left:
			pose.position.y = 1
		else:
			pose.position.y = -1
		pose.orientation = self.getOrientAsQuaternion(pose.position, self.lastBallPose.position)

		poseStamped = PoseStamped()
		poseStamped.pose = pose
		poseStamped.header.frame_id = self.namespace + "/base_footprint"
		poseStamped.header.stamp = rospy.Time.now()

		try:
			trans = self.tfBuffer.lookup_transform('map',poseStamped.header.frame_id,poseStamped.header.stamp,rospy.Duration(0.5))
		except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
			print("Fehler beim Transform der Position")
			return

		return tf2_geometry_msgs.do_transform_pose(poseStamped,trans).pose

	def goalFeedback(self,state,result):
		if state == 0 or state == 1:
			self.goalState = GoalState.active
		elif state == 2 or state == 4 or state == 6 or state == 7 or state == 8:
			self.goalState = GoalState.invalid
		elif state == 3:
			self.goalState = GoalState.reached
		elif state == 5:
			self.goalState = GoalState.invalid
		else:
			print "this should not happen"

	def stop(self):
		self.actionClient.cancel_all_goals()
		rospy.loginfo("Stopping Move Base")
		return

############### MATH ####################

	def getOrientAsQuaternion(self,fromPoint,toPoint):
		a = math.atan2(toPoint.y - fromPoint.y,toPoint.x - fromPoint.x)
		result = Quaternion()
		result.w = math.cos(a/2)
		result.z = math.sin(a/2)
		return result

	def distance(self,pointA,pointB):
		if pointA == None or pointB == None:
			return -1
		else:
			px = math.pow(pointA.x - pointB.x,2)
			py = math.pow(pointA.y - pointB.y,2)
			return math.sqrt(px+py)

	def getShootGoalPose(self):
		bp = self.lastBallPose.position
		gp = self.setup.zielTor
		d = self.distance(gp,bp)
		t = (0.5/d)
		pose = Pose()
		pose.position.x = (1+t)*bp.x-t*gp.x
		pose.position.y = (1+t)*bp.y-t*gp.y
		pose.orientation = self.getOrientAsQuaternion(pose.position, bp)
		return pose

	def turn(self,left=False):
		t = Twist()
		if left:
			t.angular.z = math.radians(90)
		else:
			t.angular.z = -math.radians(90)
#		for x in range(10):
		self.turnPub.publish(t)
		return

	def backToStart(self):
		self.gotoPose(self.setup.startPosition)
		print("Warte auf Erreichen der Starposition")
		result = self.actionClient.wait_for_result(rospy.Duration.from_sec(60))
		lastBallTime = rospy.Time(4)
		if not result:
			print("AUWEIA: Beim Aufstellen ist was schief gelaufen")
		else:
			print("Habe Startaufstellung erreicht")

	def getKickBallPose(self):
		bp = self.lastBallPose.position
		gp = self.setup.zielTor
		d = self.distance(gp, bp)
		t = 0.5/d
		pose = Pose()
		pose.position.x = (1-t)*bp.x+t*gp.x
		pose.position.y = (1-t)*bp.y+t*gp.y
		pose.orientation = self.getOrientAsQuaternion(pose.position, gp)
		return pose

	def isBallOnLeftHalf(self):
		ballTorGeradeSteigung = (self.setup.zielTor.y - self.lastBallPose.position.y) / (self.setup.zielTor.x - self.lastBallPose.position.x)
		torTorGeradeSteigung = (self.setup.zielTor.y - self.setup.eigenesTor.y) / (self.setup.zielTor.x - self.setup.eigenesTor.x)
		schnittwinkel = math.atan(math.fabs((ballTorGeradeSteigung - torTorGeradeSteigung) / (1 + (ballTorGeradeSteigung * torTorGeradeSteigung))))

		if schnittwinkel > 0:
			return False
		else:
			return True
