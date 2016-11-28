from enum import Enum

class State(Enum):
	idle = 0
	turnOnSpot = 1
	turnInCenter = 2
	drive = 3
	detour = 4
	shoot = 5
