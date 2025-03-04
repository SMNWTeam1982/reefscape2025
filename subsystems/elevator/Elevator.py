
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
from wpilib import SmartDashboard


from . import Intake
class ElevatorConstants:
    LEVEL_1_TARGET_HEIGHT = 0.1
    LEVEL_2_TARGET_HEIGHT = 0.5
    LEVEL_3_TARGET_HEIGHT = 1.0
    LEVEL_4_TARGET_HEIGHT = 1.4

    ALGAE_2_TARGET_HEIGHT = 1.2

    PROCESSOR_TARGET_HEIGHT = 0.01

    INTAKING_TARGET_HEIGHT = 0.01

    IDLE_TARGET_HEIGHT = 0.02

    ALTITUDE_PROPORTIONAL_GAIN = 0.1
    ALTITUDE_INTEGRAL_GAIN = 0.0
    ALTITUDE_DERIVATIVE_GAIN = 0.0

    MOTOR_ROTATIONS_TO_ELEVATOR_HEIGHT_MULTIPLIER = (((1.0 / 25.0) * 360.0) / 1811.0) * wpimath.units.inchesToMeters(55) + wpimath.units.inchesToMeters(1.75) # estimate from Feb 19 2025
    ELEVATOR_MAX_HEIGHT = wpimath.units.inchesToMeters(55) + wpimath.units.inchesToMeters(1.75) # 1.44145 meters

class Elevator:
    def __init__(self, pdpReference: wpilib.PowerDistribution):
        self.pdpReference = pdpReference
        self.leftAltitudeMotor = rev.SparkMax(11,rev.SparkLowLevel.MotorType.kBrushless)
        self.leftAltitudeEncoder = self.leftAltitudeMotor.getEncoder()
        self.leftAltitudeEncoder.setPositionConversionFactor(-1)

        self.rightAltitudeMotor = rev.SparkMax(12,rev.SparkLowLevel.MotorType.kBrushless)
        self.rightAltitudeEncoder = self.rightAltitudeMotor.getEncoder()

        self.altitudePIDController = wpimath.controller.PIDController(
            ElevatorConstants.ALTITUDE_PROPORTIONAL_GAIN,
            ElevatorConstants.ALTITUDE_INTEGRAL_GAIN,
            ElevatorConstants.ALTITUDE_DERIVATIVE_GAIN
        )

        self.zer0AltitudeEncoders()

        self.intake = Intake.Intake(self.pdpReference)

        self.targetHeight = ElevatorConstants.IDLE_TARGET_HEIGHT

    def zer0AltitudeEncoders(self):
        self.leftAltitudeEncoder.setPosition(0.0)
        self.rightAltitudeEncoder.setPosition(0.0)

    def getElevatorHeight(self) -> wpimath.units.meters:
        averagePosition = (self.leftAltitudeEncoder.getPosition() + self.rightAltitudeEncoder.getPosition()) / 2

        averagePosition *= ElevatorConstants.POSITION_TO_ELEVATOR_HEIGHT_MULTIPLIER

        return averagePosition + ElevatorConstants.POSITION_TO_ELEVATOR_HEIGHT_OFFSET
    
    def LogRawElevatorHeights(self):
        leftPos = self.leftAltitudeEncoder.getPosition()
        rightPos = self.rightAltitudeEncoder.getPosition()

        SmartDashboard.putNumber("raw altitude encoder left",leftPos)
        SmartDashboard.putNumber("raw altitude encoder right",rightPos)

        SmartDashboard.putNumber("average raw altitude", (leftPos + rightPos) / 2)
    
    def logElevatorHeight(self):
        SmartDashboard.putNumber("elevator height",self.getElevatorHeight())
        SmartDashboard.putNumber("target height",self.targetHeight)

    def moveElevator(self): # run pid

        amount = self.altitudePIDController.calculate(self.getElevatorHeight(),self.targetHeight)
        
        # one will need to be negated, we dont know which one yet
        self.leftAltitudeMotor.run(-amount)
        self.rightAltitudeMotor.run(amount)
    
    def runElevator(self):
        self.moveElevator()
        self.intake.runWrist()
    
    def setL1(self):
        self.targetHeight = ElevatorConstants.LEVEL_1_TARGET_HEIGHT
        self.intake.setL1()
    
    def setL2(self):
        self.targetHeight = ElevatorConstants.LEVEL_2_TARGET_HEIGHT
        self.intake.setL2()
    
    def setL3Coral(self):
        self.targetHeight = ElevatorConstants.LEVEL_3_TARGET_HEIGHT
        self.intake.setL3()
        
    def setL3Algae(self):
        self.targetHeight = ElevatorConstants.LEVEL_3_TARGET_HEIGHT
        self.intake.setAlgae()

    def setL4(self):
        self.targetHeight = ElevatorConstants.LEVEL_4_TARGET_HEIGHT
        self.intake.setL4()

    def setHighAlgae(self):
        self.targetHeight = ElevatorConstants.ALGAE_2_TARGET_HEIGHT
        self.intake.setAlgae()
    
    def setProcessor(self):
        self.targetHeight = ElevatorConstants.PROCESSOR_TARGET_HEIGHT
        self.intake.setProcessor()
    
    def setStation(self):
        self.targetHeight = ElevatorConstants.INTAKING_TARGET_HEIGHT
        self.intake.setStation()

    def setIdle(self):
        self.targetHeight = ElevatorConstants.IDLE_TARGET_HEIGHT
        self.intake.setIdle()