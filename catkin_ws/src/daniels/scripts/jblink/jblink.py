from blinkstick import blinkstick

class JanasBlinkstick(object):

	def blink(self,color,duration=500):
		for bstick in blinkstick.find_all():
			bstick.blink(name=color, repeats=2,delay=duration)

	def glow(self,color):
		for bstick in blinkstick.find_all():
			bstick.set_color(name=color)

	def off(self):
		for bstick in blinkstick.find_all():
			bstick.turn_off()		
