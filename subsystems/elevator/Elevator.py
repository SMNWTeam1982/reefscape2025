
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
    LEVEL_1_TARGET_HEIGHT = 0.56
    LEVEL_2_TARGET_HEIGHT = 0.762
    LEVEL_3_TARGET_HEIGHT = 1.1684
    LEVEL_4_TARGET_HEIGHT = 1.8

    ALGAE_2_TARGET_HEIGHT = 1.2

    PROCESSOR_TARGET_HEIGHT = 0.55

    INTAKING_TARGET_HEIGHT = 0.6

    IDLE_TARGET_HEIGHT = 0.55

    ALTITUDE_PROPORTIONAL_GAIN = 3
    ALTITUDE_INTEGRAL_GAIN = 0.0
    ALTITUDE_DERIVATIVE_GAIN = 0.0

    MOTOR_ROTATIONS_TO_ELEVATOR_HEIGHT_METERS_MULTIPLIER = ((1/118.715)*128) / 100 # march 7 2025
    ELEVATOR_HEIGHT_OFFSET = 0.53
    ELEVATOR_MAX_HEIGHT_METERS = 1.81
    ELEVATOR_MIN_HEIGHT_METERS = ELEVATOR_HEIGHT_OFFSET
    
    ALTITUDE_MOTOR_CONFIG = rev.SparkBaseConfig().smartCurrentLimit(25)

class Elevator:
    def __init__(self, pdpReference: wpilib.PowerDistribution):
        self.pdpReference = pdpReference
        self.leadMotor = rev.SparkMax(11,rev.SparkLowLevel.MotorType.kBrushless)
        self.leadMotorEncoder = self.leadMotor.getEncoder()
        self.leadMotor.configure(
            ElevatorConstants.ALTITUDE_MOTOR_CONFIG,
            rev.SparkBase.ResetMode.kResetSafeParameters,
            rev.SparkBase.PersistMode.kPersistParameters
        )

        self.followerMotor = rev.SparkMax(12,rev.SparkLowLevel.MotorType.kBrushless)
        self.followerMotorEncoder = self.followerMotor.getEncoder()
        self.followerMotor.configure(
            ElevatorConstants.ALTITUDE_MOTOR_CONFIG.follow(11, True),
            rev.SparkBase.ResetMode.kResetSafeParameters,
            rev.SparkBase.PersistMode.kPersistParameters,
        )

        self.altitudePIDController = wpimath.controller.PIDController(
            ElevatorConstants.ALTITUDE_PROPORTIONAL_GAIN,
            ElevatorConstants.ALTITUDE_INTEGRAL_GAIN,
            ElevatorConstants.ALTITUDE_DERIVATIVE_GAIN
        )

        self.zer0AltitudeEncoders()

        self.intake = Intake.Intake(self.pdpReference)

        self.targetHeight = ElevatorConstants.IDLE_TARGET_HEIGHT

        self.activeStateMachine = self.idleStateMachine # this variable IS the function, this might not be the best way to do this

    def zer0AltitudeEncoders(self):
        self.leadMotorEncoder.setPosition(0.0)
        self.followerMotorEncoder.setPosition(0.0)

    def getElevatorHeight(self) -> wpimath.units.meters:
        # negate one
        averagePosition = self.leadMotorEncoder.getPosition()

        averagePosition *= ElevatorConstants.MOTOR_ROTATIONS_TO_ELEVATOR_HEIGHT_METERS_MULTIPLIER

        return averagePosition + ElevatorConstants.ELEVATOR_HEIGHT_OFFSET
    
    def LogRawElevatorHeights(self):
        leadPos = self.leadMotorEncoder.getPosition()

        SmartDashboard.putNumber("raw altitude encoder lead motor",leadPos)
        SmartDashboard.putNumber(
            "computed altitude encoder lead motor",
            leadPos * ElevatorConstants.MOTOR_ROTATIONS_TO_ELEVATOR_HEIGHT_METERS_MULTIPLIER + ElevatorConstants.ELEVATOR_HEIGHT_OFFSET
        )    
    def logElevatorHeight(self):
        SmartDashboard.putNumber("elevator height",self.getElevatorHeight())
        SmartDashboard.putNumber("target height",self.targetHeight)
        
    def logElevatorCurrents(self):
        SmartDashboard.putNumber("right elevator current", self.followerMotor.getOutputCurrent())
        SmartDashboard.putNumber("right elevator temperature", self.followerMotor.getMotorTemperature())
        SmartDashboard.putNumber("left elevator current", self.leadMotor.getOutputCurrent())
        SmartDashboard.putNumber("left elevator temperature", self.leadMotor.getMotorTemperature())

    def moveElevator(self): # run pid
        output = -self.altitudePIDController.calculate(self.getElevatorHeight(),self.targetHeight)

        if output > 1.0:
            output = 1.0
        if output < -1.0:
            output = -1.0
        
        # one will need to be negated, we dont know which one yet
        self.leadMotor.set(-output)

    def moveElevatorRaw(self,amount: float):
        self.leadMotor.set(-amount)
    
    def moveElevatorAndWrist(self):
        self.moveElevator()
        self.intake.runWrist()
        self.intake.updateCurrentDrawHistory()

    def runStateMachine(self):
        self.activeStateMachine()

    def idleStateMachine(self):
        self.moveElevatorAndWrist()

    def coralScoreStateMachine(self):
        self.moveElevatorAndWrist()
        if self.altitudePIDController.atSetpoint() and self.intake.coralWristController.atSetpoint():
            self.intake.runIntakeEject()
            if self.intake.coralAllTheWayOut():
                self.setIdle()

    def algaeIntakeStateMachine(self):
        self.moveElevatorAndWrist()
        if self.altitudePIDController.atSetpoint() and self.intake.coralWristController.atSetpoint():
            self.intake.runIntakeEject()
            if self.intake.algaeAllTheWayIn():
                self.setIdle()

    def coralScoreAndAlgaeIntakeStateMachine(self): # for an L3 with an algae on it
        self.moveElevatorAndWrist()
        if self.altitudePIDController.atSetpoint() and self.intake.coralWristController.atSetpoint():
            self.intake.runIntakeEject()
            if self.intake.coralAllTheWayOut():
                self.setL3Algae() # after coral done set to algae intake mode
    
    def coralIntakeStateMachine(self):
        self.moveElevatorAndWrist()
        if self.altitudePIDController.atSetpoint() and self.intake.coralWristController.atSetpoint():
            self.intake.runIntakeEject()
            if self.intake.coralAllTheWayIn():
                self.setIdle()

    def algaeScoreStateMachine(self):
        self.moveElevatorAndWrist()
        if self.altitudePIDController.atSetpoint() and self.intake.coralWristController.atSetpoint():
            self.intake.runIntakeEject()
            if self.intake.algaeAllTheWayOut():
                self.setIdle()

    
    def setL1(self):
        self.targetHeight = ElevatorConstants.LEVEL_1_TARGET_HEIGHT
        self.intake.setL1()
        self.activeStateMachine = self.coralScoreStateMachine
    
    def setL2(self):
        self.targetHeight = ElevatorConstants.LEVEL_2_TARGET_HEIGHT
        self.intake.setL2()
        self.activeStateMachine = self.coralScoreStateMachine
    
    def setL3Coral(self):
        self.targetHeight = ElevatorConstants.LEVEL_3_TARGET_HEIGHT
        self.intake.setL3()
        self.activeStateMachine = self.coralScoreStateMachine
        
    def setL3Algae(self):
        self.targetHeight = ElevatorConstants.LEVEL_3_TARGET_HEIGHT
        self.intake.setAlgae()
        self.activeStateMachine = self.algaeIntakeStateMachine
    
    def setL3Combo(self):
        self.targetHeight = ElevatorConstants.LEVEL_3_TARGET_HEIGHT
        self.intake.setL3()
        self.activeStateMachine = self.coralScoreAndAlgaeIntakeStateMachine

    def setL4(self):
        self.targetHeight = ElevatorConstants.LEVEL_4_TARGET_HEIGHT
        self.intake.setL4()
        self.activeStateMachine = self.coralScoreStateMachine

    def setHighAlgae(self):
        self.targetHeight = ElevatorConstants.ALGAE_2_TARGET_HEIGHT
        self.intake.setAlgae()
        self.activeStateMachine = self.algaeIntakeStateMachine
    
    def setProcessor(self):
        self.targetHeight = ElevatorConstants.PROCESSOR_TARGET_HEIGHT
        self.intake.setProcessor()
        self.activeStateMachine = self.algaeScoreStateMachine
    
    def setStation(self):
        self.targetHeight = ElevatorConstants.INTAKING_TARGET_HEIGHT
        self.intake.setStation()
        self.activeStateMachine = self.coralIntakeStateMachine

    def setIdle(self):
        self.targetHeight = ElevatorConstants.IDLE_TARGET_HEIGHT
        self.intake.setIdle()
        self.activeStateMachine = self.idleStateMachine