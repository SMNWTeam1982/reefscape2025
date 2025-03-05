
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

        
class IntakeConstants:
    REEF_ACTIVE_TIME = 0.5
    STATION_ACTIVE_TIME = 8.0

    CORAL_INTAKE_MAX_SPEED = 0.2
    ALGAE_INTAKE_MAX_SPEED = 0.2

    LEVEL_1_CORAL_WRIST_POSITION = wpimath.geometry.Rotation2d.fromDegrees(0)
    LEVEL_MID_CORAL_WRIST_POSITION = wpimath.geometry.Rotation2d.fromDegrees(-10)
    LEVEL_4_CORAL_WRIST_POSITION = wpimath.geometry.Rotation2d.fromDegrees(-20)
    INTAKE_CORAL_WRIST_POSITION = wpimath.geometry.Rotation2d.fromDegrees(35)

    CORAL_WRIST_STARTING_POSITION = wpimath.geometry.Rotation2d.fromDegrees(-5)
    CORAL_WRIST_STOW_POSITION = wpimath.geometry.Rotation2d.fromDegrees(70) # stow up

    CORAL_ENCODER_ROTATIONS_TO_RADIANS_MULTIPLIER = math.pi/20 # Feb 15 2025


    ALGAE_PDP_CHANNEL = 11
    CORAL_PDP_CHANNEL = 13
    ALGAE_IN_CURRENT_THRESHOLD = 25
    CORAL_IN_CURRENT_THRESHOLD = 25


    CORAL_WRIST_PROPORTIONAL_GAIN = 0.05
    CORAL_WRIST_INTEGRAL_GAIN = 0.0
    CORAL_WRIST_DERIVATIVE_GAIN = 0.0


class IntakeState(enum.Enum):
    In = enum.auto
    Out = enum.auto
    Hold = enum.auto

class Intake:
    def __init__(self, pdpReference: wpilib.PowerDistribution):
        self.pdpReference = pdpReference

        self.coralWristMotor = rev.SparkMax(15,rev.SparkLowLevel.MotorType.kBrushless)
        self.coralWristEncoder = self.coralWristMotor.getEncoder()

        self.coralMotor = rev.SparkMax(16,rev.SparkLowLevel.MotorType.kBrushless)

        self.rightAlgaeMotor = rev.SparkMax(14,rev.SparkLowLevel.MotorType.kBrushless)
        self.leftAlgaeMotor = rev.SparkMax(13,rev.SparkLowLevel.MotorType.kBrushless)

        # self.cooldownTimer = wpilib.Timer()

        self.coralWristController = wpimath.controller.PIDController(
            IntakeConstants.CORAL_WRIST_PROPORTIONAL_GAIN,
            IntakeConstants.CORAL_WRIST_INTEGRAL_GAIN,
            IntakeConstants.CORAL_WRIST_DERIVATIVE_GAIN
        )
        
        self.coralWristTarget = IntakeConstants.CORAL_WRIST_STARTING_POSITION

        self.algaeIntakeState = IntakeState.Hold
        self.coralIntakeState = IntakeState.Hold
    
    def zeroEncoder(self):
        self.coralWristEncoder.setPosition(0.0)

    # returns true if intake/ejection is complete
    # def runIntakesTimed(self) -> bool:
    #     if self.coralIntakeState == IntakeState.In:
    #         self.runIntakeEject()
    #         return False


    #     if self.coralIntakeState != IntakeState.Hold or self.algaeIntakeState != IntakeState.Hold:
    #         if self.cooldownTimer.isRunning():
    #             if self.cooldownTimer.get() < IntakeConstants.ACTIVE_TIME:
    #                 self.runIntakeEject()
    #                 return False
    #             else:
    #                 self.cooldownTimer.stop()
    #                 self.cooldownTimer.reset()
    #                 return True
    #         else:
    #             self.cooldownTimer.start()
    #             return False
    #     else:
    #         self.coralMotor.stopMotor()
    #         self.leftAlgaeMotor.stopMotor()
    #         self.rightAlgeaMotor.stopMotor()
    #         return True

    def algaeAllTheWayIn(self) -> bool:
        return self.pdpReference.getCurrent(IntakeConstants.ALGAE_PDP_CHANNEL) > IntakeConstants.ALGAE_IN_CURRENT_THRESHOLD
    
    def coralAllTheWayIn(self) -> bool:
        return self.pdpReference.getCurrent(IntakeConstants.CORAL_PDP_CHANNEL) > IntakeConstants.CORAL_IN_CURRENT_THRESHOLD


    def runWrist(self):
        coralAmount = self.coralWristController.calculate(
            self.coralWristEncoder.getPosition() * IntakeConstants.CORAL_ENCODER_ROTATIONS_TO_RADIANS_MULTIPLIER,
            self.coralWristTarget.radians()
        )

        self.coralWristMotor.set(coralAmount)

    def runIntakeEject(self):
        if self.coralIntakeState == IntakeState.In:
            self.coralMotor.set(-IntakeConstants.CORAL_INTAKE_MAX_SPEED)
            if self.coralAllTheWayIn:
                self.setIdle()
        if self.coralIntakeState == IntakeState.Out:
            self.coralMotor.set(IntakeConstants.CORAL_INTAKE_MAX_SPEED)
        if self.coralIntakeState == IntakeState.Hold:
            self.coralMotor.set(0.0)

        if self.algaeIntakeState == IntakeState.In:
            self.leftAlgaeMotor.set(-IntakeConstants.ALGAE_INTAKE_MAX_SPEED)
            self.rightAlgaeMotor.set(IntakeConstants.ALGAE_INTAKE_MAX_SPEED)
            if self.algaeAllTheWayIn():
                self.setIdle()
        if self.algaeIntakeState == IntakeState.Out:
            self.leftAlgaeMotor.set(IntakeConstants.ALGAE_INTAKE_MAX_SPEED)
            self.rightAlgaeMotor.set(-IntakeConstants.ALGAE_INTAKE_MAX_SPEED)
        if self.algaeIntakeState == IntakeState.Hold:
            self.leftAlgaeMotor.set(0.0)
            self.rightAlgaeMotor.set(0.0)

    # def suspendTask(self):
    #     self.cooldownTimer.stop()
    #     self.cooldownTimer.reset()

    #     self.coralMotor.stopMotor()
    #     self.leftAlgaeMotor.stopMotor()
    #     self.rightAlgeaMotor.stopMotor()

    def logIntakeCurrents(self):
        SmartDashboard.putNumber("algae current (left motor)", self.pdpReference.getCurrent(IntakeConstants.ALGAE_PDP_CHANNEL))
        SmartDashboard.putNumber("coral current", self.pdpReference.getCurrent(IntakeConstants.CORAL_PDP_CHANNEL))

    def logWristPosition(self):
        SmartDashboard.putNumber("coralWristPosition", self.coralWristEncoder.getPosition() * IntakeConstants.CORAL_ENCODER_ROTATIONS_TO_RADIANS_MULTIPLIER)


    
    def setL1(self):
        self.coralWristTarget = IntakeConstants.LEVEL_1_CORAL_WRIST_POSITION
        self.algaeIntakeState = IntakeState.Hold
        self.coralIntakeState = IntakeState.Out
        # self.suspendTask()

    def setL2(self):
        self.coralWristTarget = IntakeConstants.LEVEL_MID_CORAL_WRIST_POSITION
        self.algaeIntakeState = IntakeState.Hold
        self.coralIntakeState = IntakeState.Out
        # self.suspendTask()
 
    def setL3(self):
        self.coralWristTarget = IntakeConstants.LEVEL_MID_CORAL_WRIST_POSITION
        self.algaeIntakeState = IntakeState.In
        self.coralIntakeState = IntakeState.Out
        # self.suspendTask()
 
    def setL4(self):
        self.coralWristTarget = IntakeConstants.LEVEL_4_CORAL_WRIST_POSITION
        self.algaeIntakeState = IntakeState.Hold
        self.coralIntakeState = IntakeState.Out
        # self.suspendTask()
    
    def setAlgae(self):
        self.coralWristTarget = IntakeConstants.CORAL_WRIST_STOW_POSITION
        self.algaeIntakeState = IntakeState.In
        self.coralIntakeState = IntakeState.Hold
        # self.suspendTask()
    
    def setProcessor(self):
        self.coralWristTarget = IntakeConstants.CORAL_WRIST_STOW_POSITION
        self.algaeIntakeState = IntakeState.Out
        self.coralIntakeState = IntakeState.Hold
        # self.suspendTask()
 
    def setStation(self):
        self.coralWristTarget = IntakeConstants.INTAKE_CORAL_WRIST_POSITION
        self.algaeIntakeState = IntakeState.Hold
        self.coralIntakeState = IntakeState.In
        # self.suspendTask()
    
    def setIdle(self):
        self.coralWristTarget = IntakeConstants.CORAL_WRIST_STOW_POSITION
        self.algaeIntakeState = IntakeState.Hold
        self.coralIntakeState = IntakeState.Hold
        # self.suspendTask()