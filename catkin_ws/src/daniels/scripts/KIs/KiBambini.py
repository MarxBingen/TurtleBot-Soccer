from base.KiBase import KiBase
from geometry_msgs.msg import PoseStamped, Pose

class KiBambini(KiBase):

	def __init__(self):
		super(KiBambini,self).__init__()

	def ballDetected(self, ballPosition):
		self.base().ballDetected(ballPosition)
		return

	def setupUpdated(self, setup):
		self.base().setupUpdated(setup)
		print("hab n neues Setup bekommen")
		return

	def update(self,time):
		self.base().update(time)
		t = self.timeSinceLastBall()
		if (t > 3 and t < 20):
			print "Drehe mich, um Ball zu finden"
			self.turn()
		elif t >= 20:
			self.backToStart()
		else:
			print "GA?" + str(self.goal_active)
			if not self.goal_active:
				self.myShotBall()
			else:
				print "Diff:" + str(self.lastBallPoseDiff)
				if self.lastBallPoseDiff > 0.5:
					self.myShotBall()
		return

	def myShotBall(self):
		newPos = self.lastBallPose
		o = self.base().myPos.position
		newPos.pose.orientation = self.base().getOrientAsQuaternion(newPos.pose.position,self.setup.zielTor)
		self.base().gotoPos(newPos)

	def base(self):
		return super(KiBambini,self)
