
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


        
class IntakeConstants:
    CORAL_INTAKE_MAX_SPEED = 0.0

    LEVEL_1_WRIST_POSITION = 0.0
    LEVEL_MID_WRIST_POSITION = 0.0
    LEVEL_4_WRIST_POSITION = 0.0

    ALGAE_INTAKE_MAX_SPEED = 0.0

    ALGAE_OUT = 0.0

    STOW = 0.0

    WRIST_PROPORTIONAL_GAIN = 1.0
    WRIST_INTEGRAL_GAIN = 0.0
    WRIST_DERIVATIVE_GAIN = 0.0

class WristTarget(enum.Enum):

    L1 = IntakeConstants.LEVEL_1_WRIST_POSITION
    LMID = IntakeConstants.LEVEL_MID_WRIST_POSITION
    L4 = IntakeConstants.LEVEL_4_WRIST_POSITION

    CoralMax = IntakeConstants.CORAL_INTAKE_MAX_SPEED
    AlgaeMax = IntakeConstants.ALGAE_INTAKE_MAX_SPEED

    AlgaeOut = IntakeConstants.ALGAE_OUT

    Stow = IntakeConstants.STOW


class IntakeState(enum.Enum):
    Disabled = enum.auto
    Moving = enum.auto


class Intake:
    def __init__(self):
        self.wristMotor = rev.CANSparkMax(0,rev.SparkLowLevel.MotorType.kBrushless)
        self.wristEncoder = self.WristMotor.getEncoder()

        self.rightAlgeaMotor = rev.CANSparkMax(0,rev.SparkLowLevel.MotorType.kBrushless)

        self.leftAlgaeMotor = rev.CANSparkMax(0,rev.SparkLowLevel.MotorType.kBrushless)

        self.coralMotor = rev.CANSparkMax(0,rev.SparkLowLevel.MotorType.kBrushless)


        self.wristPIDController = wpimath.controller.PIDController(
            IntakeConstants.WRIST_PROPORTIONAL_GAIN,
            IntakeConstants.WRIST_INTEGRAL_GAIN,
            IntakeConstants.WRIST_DERIVATIVE_GAIN
        )

        self.state = IntakeState.Stopped
        self.target = WristTarget.Intake


    def getWristPosition(self) -> wpimath.geometry.Rotation2d:
        wristPosition = self.wristEncoder.getPosition() # convert to rotation 2d when we get the actual encoder information

        return wristPosition 
    
    def moveWrist(self):
        amount = self.wristPIDController.calculate(self.getWristPosition(),self.state.value)

        self.wristMotor.run(amount)

    def runWristController(self):
        match self.state:
            case IntakeState.Disabled:
                self.wristMotor.stop()
            case IntakeState.Moving:
                self.moveWrist()
            case _:
                # this case should never be reached, we should put something up on telemetry
                self.state = IntakeState.Stopped

    def setState(self,state: IntakeState):
        self.state = state

    def wristTarget(self,target: WristTarget):
        self.target = target


