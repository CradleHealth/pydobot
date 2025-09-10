from enum import Enum

class Suction(Enum):
    OPEN = 0      # release (vacuum off)
    CLOSE = 1     # hold (vacuum on)
    DISABLE = 2   # controller disabled