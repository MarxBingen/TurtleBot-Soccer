from base.KiBase import KiBase
from geometry_msgs.msg import Pose
from StatesVerteidiger import State
from base.GoalStates import GoalState
import rospy


class KiVerteidiger(KiBase):


	state = State.idle
	lastDest = None
	onSpotTime = rospy.Time()
	detourPose = None
	driveAfterDetour = False
	kickBallPose = None


	def __init__(self):
		super(KiVerteidiger, self).__init__()


	def stop(self):
		super(KiVerteidiger, self).stop()
		self.state = State.idle
		self.lastDest = None
		self.kickBallPose = None
		self.detourPose = None
		self.driveAfterDetour = False


	def bleibStehen(self):
		self.actionClient.cancel_all_goals()


	def ballCloserToGoal(self):
		n = self.distance(self.setup.zielTor, self.lastBallPose.position)
		m = self.distance(self.setup.zielTor, self.myPose.position)
		print "Tor"
		print self.setup.zielTor
		print "Ball"
		print self.lastBallPose.position
		print "Bot"
		print self.myPose.position
		print n
		print m
		return n < m

	def ballOnOwnSide(self):
		return self.distance(self.setup.eigenesTor, self.lastBallPose.position) < self.distance(self.setup.eigenesTor, self.center)


	def update(self, time):
		super(KiVerteidiger, self).update(time)
		print "NEXT UPDATE:"


		if self.state == State.idle:

			print "STATE: IDLE"

			if self.timeSinceLastBall() > 3:

				self.onSpotTime = rospy.Time.now()
				self.state = State.turnOnSpot
				print "NO BALL DETECTED, GOING TO TURNON SPOT"

			else:

				if self.ballOnOwnSide():

					self.state = State.drive
					print "BALL ON MY SIDE DETECTED, GOING TO DRIVE"

				else:

					print "WAITING FOR ATTACK"


		elif self.state == State.turnOnSpot:

			print "STATE: TURN ON SPOT"

			if self.timeSinceLastBall() > 3:

				if rospy.Time.now().secs - self.onSpotTime.secs > 10:

					self.bleibStehen()
					self.goalState = GoalState.none
					self.state = State.backToGoal
					print "NO BALL DETECTED, GOING BACK TO GOAL"

				else:

					self.turn()
					print "NO BALL DETECTED, TURNING ON SPOT"

			else:

				if self.ballOnOwnSide():

					self.bleibStehen()
					self.goalState = GoalState.none
					self.state = State.drive
					print "BALL ON MY SIDE DETECTED, GOING TO DRIVE"

				else:

					self.bleibStehen()
					self.goalState = GoalState.none
					self.state = State.backToGoal
					print "BALL ON OTHER SIDE, GOING BACK TO GOAL"


		elif self.state == State.backToGoal:

			print "STATE: BACK TO GOAL"

			if self.lastDest == None:

				self.lastDest = self.setup.startPosition

				if self.timeSinceLastBall < 3 and self.distance(self.myPose.position, self.lastDest.position) > self.distance(self.lastBallPose.position, self.lastDest.position):

					self.bleibStehen()
					self.goalState = GoalState.none
					self.state = State.detour
					print "HAVE TO TAKE A DETOUR TO GOAL"

				else:

					self.gotoPose(self.lastDest)
					print "GOING TO GOAL"

			elif self.timeSinceLastBall < 3 and self.distance(self.myPose.position, self.lastDest.position) > self.distance(self.lastBallPose.position, self.lastDest.position):

				self.bleibStehen()
				self.goalState = GoalState.none
				self.state = State.detour
				print "HAVE TO TAKE A DETOUR TO GOAL"

			elif self.goalState == GoalState.reached:

				self.bleibStehen()
				self.goalState = GoalState.none
				self.lastDest = None
				self.state = State.idle
				print "GOAL REACHED"

			else:

				print "DRIVING ON TO GOAL"


		elif self.state == State.drive:

			print "STATE: DRIVE"

			if self.timeSinceLastBall() > 3:

				self.bleibStehen()
				self.goalState = GoalState.none
				self.lastDest = None
				self.onSpotTime = rospy.Time.now()
				self.state = State.turnOnSpot
				print "NO BALL DETECTED, GOING TO TURN ON SPOT"

			elif self.lastDest == None or self.lastBallPoseDiff > 0.5:

				self.lastDest = self.getShootGoalPose()

				if self.distance(self.myPose.position, self.lastDest.position) < 0.5 and self.ballCloserToGoal():

					self.lastDest.position = self.myPose.position
					self.gotoPose(self.lastDest)
					self.bleibStehen()
					self.lastDest = None
					self.goalState = GoalState.none
					self.state = State.shoot
					print "ALREADY AT SHOOTING POSITION"

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
				self.goalState = GoalState.none
				self.lastDest = None
				self.state = State.backToGoal
				print "WANTED TO DRIVE BUT GOAL INVALID"

			elif self.goalState == GoalState.reached:

				self.bleibStehen()
				self.goalState = GoalState.none
				self.lastDest = None
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
						self.goalState = GoalState.none
						self.lastDest = None
						self.detourPose = None
						self.driveAfterDetour = False
						self.state = State.backToGoal
						print "WANTED TO DRIVE A DETOUR BUT GOAL INVALID"

					elif self.goalState == GoalState.reached:

						self.bleibStehen()
						self.goalState = GoalState.none
						self.driveAfterDetour = True
						self.detourPose = None
						self.gotoPose(self.lastDest)
						print "DETOUR DESTINATION REACHED, DRIVING TO SHOOTING POSITION"

					else:

						print "STILL DRIVING DETOUR"

				else:

					if self.timeSinceLastBall() > 3:

						self.bleibStehen()
						self.goalState = GoalState.none
						self.lastDest = None
						self.driveAfterDetour = False
						self.detourPose = None
						self.state = State.backToGoal

					elif self.goalState == GoalState.invalid:

						self.bleibStehen()
						self.goalState = GoalState.none
						self.lastDest = None
						self.driveAfterDetour = False
						self.state = State.backToGoal
						print "WANTED TO DRIVE TO SHOOTING POSITION BUT GOAL INVALID"

					elif self.goalState == GoalState.reached:

						self.bleibStehen()
						self.goalState = GoalState.none
						self.lastDest = None
						self.driveAfterDetour = False
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
				self.state = State.backToGoal
				print "BALL KICKED"

			elif self.goalState == GoalState.invalid:

				self.bleibStehen()
				self.goalState = GoalState.none
				self.kickBallPose = None
				self.state = State.backToGoal
				print "WANTED TO SHOOT BALL, BUT GOAL INVALID"

			else:

				print "STILL SHOOTING"
