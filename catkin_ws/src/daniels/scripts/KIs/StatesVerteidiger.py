from enum import Enum

class State(Enum):
	idle = 0
	turnOnSpot = 1
	drive = 2
	shoot = 3
	backToGoal = 4
	detour = 5
