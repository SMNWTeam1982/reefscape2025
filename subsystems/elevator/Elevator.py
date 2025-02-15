
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



from Intake import Intake
from Intake import IntakeState
class ElevatorConstants:
    LEVEL_1_TARGET_HEIGHT = 0.0
    LEVEL_2_TARGET_HEIGHT = 0.0
    LEVEL_3_TARGET_HEIGHT = 0.0
    LEVEL_4_TARGET_HEIGHT = 0.0

    ALGAE_2_TARGET_HEIGHT = 0.0

    PROCESSOR_TARGET_HEIGHT = 0.0

    INTAKING_TARGET_HEIGHT = 0.0

    IDLE_TARGET_HEIGHT = 0.0

    ALTITUDE_PROPORTIONAL_GAIN = 1.0
    ALTITUDE_INTEGRAL_GAIN = 0.0
    ALTITUDE_DERIVATIVE_GAIN = 0.0

    POSITION_TO_ELEVATOR_HEIGHT_MULTIPLIER = 1.0
    POSITION_TO_ELEVATOR_HEIGHT_OFFSET = 0.0

class Elevator:
    def __init__(self):
        self.leftAltitudeMotor = rev.SparkMax(0,rev.CANSparkLowLevel.MotorType.kBrushless)
        self.leftAltitudeEncoder = self.leftAltitudeMotor.getEncoder()

        self.rightAltitudeMotor = rev.SparkMax(0,rev.CANSparkLowLevel.MotorType.kBrushless)
        self.rightAltitudeEncoder = self.rightAltitudeMotor.getEncoder()

        self.altitudePIDController = wpimath.controller.PIDController(
            ElevatorConstants.ALTITUDE_PROPORTIONAL_GAIN,
            ElevatorConstants.ALTITUDE_INTEGRAL_GAIN,
            ElevatorConstants.ALTITUDE_DERIVATIVE_GAIN
        )

        self.intake = Intake()

        self.targetHeight = ElevatorConstants.IDLE_TARGET_HEIGHT

    def getElevatorHeight(self) -> wpimath.units.meters:
        # one of these positions will need to be negated before adding, we dont know which one yet
        averagePosition = (self.leftAltitudeEncoder.getPosition() + self.rightAltitudeEncoder.getPosition()) / 2

        averagePosition *= ElevatorConstants.POSITION_TO_ELEVATOR_HEIGHT_MULTIPLIER

        return averagePosition + ElevatorConstants.POSITION_TO_ELEVATOR_HEIGHT_OFFSET
    
    def moveElevator(self): # run pid

        amount = self.altitudePIDController.calculate(self.getElevatorHeight(),self.targetHeight)
        
        # one will need to be negated, we dont know which one yet
        self.leftAltitudeMotor.run(-amount)
        self.rightAltitudeMotor.run(amount)
    
    def runElevator(self):
        self.moveElevator()
        self.intake.runWrists()

        if not self.altitudePIDController.atSetpoint():
            return
        if self.targetHeight == ElevatorConstants.INTAKING_TARGET_HEIGHT:
            self.intake.runIntakeEject()