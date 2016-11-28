from enum import Enum

class GoalState(Enum):
	none = 0
	active = 1
	cancelled = 2
	reached = 3
	invalid = 4
