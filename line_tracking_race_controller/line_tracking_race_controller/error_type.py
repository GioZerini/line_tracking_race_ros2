from enum import Enum


# Enum defining different error types of the PID
class ErrorType(Enum):
    OFFSET = 0
    ANGLE = 1