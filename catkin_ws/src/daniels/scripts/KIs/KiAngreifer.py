from base.KiBase import KiBase
from geometry_msgs.msg import Pose
from StatesAngreifer import State
from base.GoalStates import GoalState
import rospy
import math


class KiAngreifer(KiBase):


	state = State.idle
	lastDest = None
	onSpotTime = rospy.Time()
	detourPose = None
	driveAfterDetour = False
	kickBallPose = None
	turning = False


	def __init__(self):
		super(KiAngreifer, self).__init__()


	def stop(self):
		super(KiAngreifer, self).stop()
		self.state = State.idle
		self.lastDest = None
		self.kickBallPose = None
		self.detourPose = None
		self.driveAfterDetour = False
		self.turning = False


	def bleibStehen(self):
		self.actionClient.cancel_all_goals()


	def ballCloserToGoal(self):
		return self.distance(self.setup.zielTor, self.lastBallPose.position) < self.distance(self.setup.zielTor, self.myPose.position)


	def update(self, time):
		super(KiAngreifer, self).update(time)
		print "NEXT UPDATE:"


		if self.state == State.idle:

			print "STATE: IDLE"

			if self.timeSinceLastBall() > 3:

				self.onSpotTime = rospy.Time.now()
				self.state = State.turnOnSpot
				print "NO BALL DETECTED, GOING TO TURN ON SPOT"

			else:

				self.state = State.drive
				print "BALL DETECTED, GOING TO DRIVE"


		elif self.state == State.turnOnSpot:

			print "STATE: TURN ON SPOT"

			if self.timeSinceLastBall() > 3:

				if rospy.Time.now().secs - self.onSpotTime.secs > 10:

					self.bleibStehen()
					self.goalState = GoalState.none
					self.state = State.turnInCenter
					print "NO BALL DETECTED, GOING TO TURN IN CENTER"

				else:

					self.turn()
					print "NO BALL DETECTED, TURNING ON SPOT"

			else:

				self.bleibStehen()
				self.goalState = GoalState.none
				self.state = State.drive
				print "BALL DETECTED, GOING TO DRIVE"


		elif self.state == State.turnInCenter:

			print "STATE: TURN IN CENTER"

			if self.timeSinceLastBall() > 3:

				if self.lastDest == None:

					self.lastDest = Pose()
					self.lastDest.position = self.center
					self.lastDest.orientation = self.getOrientAsQuaternion(self.center, self.setup.zielTor)

					if self.distance(self.myPose.position, self.center) > 0.3:

						self.gotoPose(self.lastDest)
						print "NO BALL DETECTED, DRIVING TO CENTER"

					else:

						self.turning = True
						print "NO BALL DETECTED,ALREADY IN CENTER, GOING TO TURN IN CENTER"

				elif self.turning:

					self.turn()
					print "NO BALL DETECTED, TURNING IN CENTER"

				elif self.goalState == GoalState.reached:

					self.turning = True
					print "NO BALL DETECTED,CENTER REACHED, GOING TO TURN"

				else:

					print "STILL DRIVING TO CENTER"

			else:

				self.bleibStehen()
				self.lastDest = None
				self.goalState = GoalState.none
				self.turning = False
				self.state = State.drive
				print "BALL DETECTED, GOING TO DRIVE"


		elif self.state == State.drive:

			print "STATE: DRIVE"

			if self.timeSinceLastBall() > 3:

				self.bleibStehen()
				self.lastDest = None
				self.goalState = GoalState.none
				self.onSpotTime = rospy.Time.now()
				self.state = State.turnOnSpot
				print "NO BALL DETECTED, GOING TO TURN ON SPOT"

			elif self.lastDest == None or self.lastBallPoseDiff > 0.5:

				self.lastDest = self.getShootGoalPose()

				if self.distance(self.myPose.position, self.lastBallPose.position) < 0.5 and self.ballCloserToGoal():

					self.lastDest.position = self.myPose.position
					self.gotoPose(self.lastDest)
					self.bleibStehen()
					self.lastDest = None
					self.goalState = GoalState.none
					self.state = State.shoot
					print "ALREADY AT SHOOTING POSITION, GOING TO SHOOT"
					print self.getOrientAsQuaternion(self.myPose.position, self.setup.zielTor)
#					print self.getOrientAsQuaternion(self.myPose.position, self.setup.zielTor)
					print self.getOrientAsQuaternion(self.myPose.position, self.lastBallPose.position)
#					print self.getOrientAsQuaternion(self.myPose.position, self.lastBallPose.position)

				elif not self.ballCloserToGoal():

					self.bleibStehen()
					self.goalState = GoalState.none
					self.state = State.detour
					print "HAVE TO TAKE A DETOUR"

				else:

					self.gotoPose(self.lastDest)
					print "GOING TO SHOOTING POSITION"

			elif not self.ballCloserToGoal():

				self.bleibStehen()
				self.goalState = GoalState.none
				self.state = State.detour
				print "HAVE TO TAKE A DETOUR"


			elif self.goalState == GoalState.invalid:

				self.bleibStehen()
				self.lastDest = None
				self.goalState = GoalState.none
				self.state = State.idle
				print "WANTED TO DRIVE BUT GOAL INVALID"

			elif self.goalState == GoalState.reached:

				self.bleibStehen()
				self.lastDest = None
				self.goalState = GoalState.none
				self.state = State.shoot
				print "SHOOTING POSITION REACHED"

			else:

				print "DRIVING ON TO SHOOTNG POSITION"


		elif self.state == State.detour:

			print "STATE: DETOUR"

			if self.detourPose == None:

				self.detourPose = self.getMeterFromBot()
				self.gotoPose(self.detourPose)
				print "DRIVING A DETOUR"

			else:

				if not self.driveAfterDetour:

					if self.goalState == GoalState.invalid:

						self.bleibStehen()
						self.lastDest = None
						self.goalState = GoalState.none
						self.detourPose = None
						self.driveAfterDetour = False
						self.state = State.idle
						print "WANTED TO DRIVE A DETOUR BUT GOAL INVALID"

					elif self.goalState == GoalState.reached:

						self.bleibStehen()
						self.goalState = GoalState.none
						self.detourPose = None
						self.driveAfterDetour = True
						self.gotoPose(self.lastDest)
						print "DETOUR DESTINATION REACHED, DRIVING TO SHOOTING POSITION"

					else:

						print "STILL DRIVING DETOUR"

				else:

					if self.timeSinceLastBall() > 3:

						self.bleibStehen()
						self.lastDest = None
						self.goalState = GoalState.none
						self.driveAfterDetour = False
						self.detourPose = None
						self.onSpotTime = rospy.Time.now()
						self.state = State.turnOnSpot
						print "NO BALL DETECTED, GOING TO TURN ON SPOT"

					elif self.goalState == GoalState.invalid:

						self.bleibStehen()
						self.lastDest = None
						self.goalState = GoalState.none
						self.driveAfterDetour = False
						self.state = State.idle
						print "WANTED TO DRIVE TO SHOOTING POSITION BUT GOAL INVALID"

					elif self.goalState == GoalState.reached:

						self.bleibStehen()
						self.lastDest = None
						self.driveAfterDetour = False
						self.goalState = GoalState.none
						self.state = State.shoot
						print "SHOOTING POSITION AFTER DETOUR REACHED"

					else:

						print "STILL DRIVING TO SHOOTING POSITION AFTER DETOUR"


		elif self.state == State.shoot:

			print "STATE: SHOOT"

			if self.timeSinceLastBall() > 3:

				self.bleibStehen()
				self.goalState = GoalState.none
				self.kickBallPose = None
				self.onSpotTime = rospy.Time.now()
				self.state = State.turnOnSpot
				print "BALL NOT KICKED, NO BALL DETECTED, GOING TO TURN"

			elif self.kickBallPose == None:

				self.kickBallPose = self.getKickBallPose()
				self.gotoPose(self.kickBallPose)
				print "SHOOTING"

			elif self.goalState == GoalState.reached:

				self.bleibStehen()
				self.goalState = GoalState.none
				self.kickBallPose = None
				self.state = State.idle
				print "BALL KICKED"

			elif self.goalState == GoalState.invalid:

				self.bleibStehen()
				self.goalState = GoalState.none
				self.kickBallPose = None
				self.state = State.idle
				print "WANTED TO SHOOT BALL BUT GOAL INVALID"

			else:

				print "STILL SHOOTING"
