
import math
import enum
import wpilib
import wpimath.kinematics
import wpimath.geometry
import wpimath.controller
import wpimath.trajectory

import rev
from phoenix6 import hardware as ctre
import wpimath.units

class ElevatorConstants:
    LEVEL_1_TARGET_HEIGHT = 0.0
    LEVEL_2_TARGET_HEIGHT = 0.0
    LEVEL_3_TARGET_HEIGHT = 0.0
    LEVEL_4_TARGET_HEIGHT = 0.0

    INTAKING_TARGET_HEIGHT = 0.0

    ALTITUDE_PROPORTIONAL_GAIN = 1.0
    ALTITUDE_INTEGRAL_GAIN = 0.0
    ALTITUDE_DERIVATIVE_GAIN = 0.0

    POSITION_TO_ELEVATOR_HEIGHT_MULTIPLIER = 1.0
    POSITION_TO_ELEVATOR_HEIGHT_OFFSET = 0.0

class ElevatorState(enum.Enum):
    Stopped = enum.auto
    Moving = enum.auto
    Intaking = enum.auto
    DispenseL1 = enum.auto
    DispenseL4 = enum.auto
    DispenseOtherLevels = enum.auto
class AltitudeTarget(enum.Enum):
    L1 = ElevatorConstants.LEVEL_1_TARGET_HEIGHT
    L2 = ElevatorConstants.LEVEL_2_TARGET_HEIGHT
    L3 = ElevatorConstants.LEVEL_3_TARGET_HEIGHT
    L4 = ElevatorConstants.LEVEL_4_TARGET_HEIGHT

    Intake = ElevatorConstants.INTAKING_TARGET_HEIGHT



class Elevator:
    def __init__(self):
        self.leftAltitudeMotor = rev.CANSparkMax(0,rev.CANSparkLowLevel.MotorType.kBrushless)
        self.leftAltitudeEncoder = self.leftAltitudeMotor.getEncoder()

        self.rightAltitudeMotor = rev.CANSparkMax(0,rev.CANSparkLowLevel.MotorType.kBrushless)
        self.rightAltitudeEncoder = self.rightAltitudeMotor.getEncoder()

        self.altitudePIDController = wpimath.controller.PIDController(
            ElevatorConstants.ALTITUDE_PROPORTIONAL_GAIN,
            ElevatorConstants.ALTITUDE_INTEGRAL_GAIN,
            ElevatorConstants.ALTITUDE_DERIVATIVE_GAIN
        )

        self.state = ElevatorState.Stopped
        self.target = AltitudeTarget.Intake

    def getElevatorHeight(self) -> wpimath.units.meters:

        # one of these positions will need to be negated before adding, we dont know which one yet
        averagePosition = (self.leftAltitudeEncoder.getPosition() + self.rightAltitudeEncoder.getPosition()) / 2

        averagePosition *= ElevatorConstants.POSITION_TO_ELEVATOR_HEIGHT_MULTIPLIER

        return averagePosition + ElevatorConstants.POSITION_TO_ELEVATOR_HEIGHT_OFFSET
    
    def moveElevator(self):

        amount = self.altitudePIDController.calculate(self.getElevatorHeight(),self.state.value)
        
        # one will need to be negated, we dont know which one yet
        self.leftAltitudeMotor.run(amount)
        self.rightAltitudeMotor.run(amount)

    def runAltitudeController(self):
        match self.state:
            case ElevatorState.Stopped:
                self.leftAltitudeMotor.stop()
                self.rightAltitudeMotor.stop()
            case ElevatorState.Moving:
                self.moveElevator()
            case _:
                # this case should never be reached, we should put something up on telemetry
                self.state = ElevatorState.Stopped

    def setState(self,state: ElevatorState):
        self.state = state
