
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

    MOTOR_ROTATIONS_TO_ELEVATOR_HEIGHT_MULTIPLIER = ((1.0 / 25.0) * 360.0) / 1811.0 # estimate from Feb 19 2025
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

        self.zer0AltitudeEncoders()

        self.intake = Intake()

        self.targetHeight = ElevatorConstants.IDLE_TARGET_HEIGHT

    def zer0AltitudeEncoders(self):
        self.leftAltitudeEncoder.setPosition(0.0)
        self.rightAltitudeEncoder.setPosition(0.0)

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
        elif self.intake.runIntakesTimed(): # will run the intakes and check if they are done
            self.targetHeight = ElevatorConstants.IDLE_TARGET_HEIGHT
            self.intake.setIdle() # ensure that once intake is done that you set it back to idle mode
    
    def setL1(self):
        self.targetHeight = ElevatorConstants.LEVEL_1_TARGET_HEIGHT
        self.intake.setL1()
    
    def setL2(self):
        self.targetHeight = ElevatorConstants.LEVEL_2_TARGET_HEIGHT
        self.intake.setL2()
    
    def setL3(self):
        self.targetHeight = ElevatorConstants.LEVEL_3_TARGET_HEIGHT
        self.intake.setL3()
    
    def setL4(self):
        self.targetHeight = ElevatorConstants.LEVEL_4_TARGET_HEIGHT
        self.intake.setL4()

    def setHighAlgae(self):
        self.targetHeight = ElevatorConstants.ALGAE_2_TARGET_HEIGHT
        self.intake.setHighAlgae()
    
    def setProcessor(self):
        self.targetHeight = ElevatorConstants.PROCESSOR_TARGET_HEIGHT
        self.intake.setProcessor()
    
    def setStation(self):
        self.targetHeight = ElevatorConstants.INTAKING_TARGET_HEIGHT
        self.intake.setStation()

    def setIdle(self):
        self.targetHeight = ElevatorConstants.IDLE_TARGET_HEIGHT
        self.intake.setIdle()