
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
    REEF_ACTIVE_TIME = 0.5
    STATION_ACTIVE_TIME = 8.0

    CORAL_INTAKE_MAX_SPEED = 0.2
    ALGAE_INTAKE_MAX_SPEED = 0.2

    LEVEL_1_CORAL_WRIST_POSITION = wpimath.geometry.Rotation2d.fromDegrees(0)
    LEVEL_MID_CORAL_WRIST_POSITION = wpimath.geometry.Rotation2d.fromDegrees(0)
    LEVEL_4_CORAL_WRIST_POSITION = wpimath.geometry.Rotation2d.fromDegrees(0)
    INTAKE_CORAL_WRIST_POSITION = wpimath.geometry.Rotation2d.fromDegrees(0)

    CORAL_WRIST_STARTING_POSITION = wpimath.geometry.Rotation2d.fromDegrees(0)
    CORAL_WRIST_STOW_POSITION = wpimath.geometry.Rotation2d.fromDegrees(0)

    CORAL_ENCODER_ROTATIONS_TO_RADIANS_MULTIPLIER = math.pi/20 # Feb 15 2025

    ALGAE_WRIST_INTAKE_POSITION = wpimath.geometry.Rotation2d.fromDegrees(0)
    ALGAE_WRIST_EJECT_POSITION = wpimath.geometry.Rotation2d.fromDegrees(0)
    ALGAE_WRIST_STOW_POSITION = wpimath.geometry.Rotation2d.fromDegrees(0)

    ALGAE_WRIST_STARTING_POSITION = wpimath.geometry.Rotation2d.fromDegrees(0)
    
    ALGAE_ENCODER_ROTATIONS_TO_RADIANS_MULTIPLIER = math.pi/20 # Feb 15 2025


    CORAL_WRIST_PROPORTIONAL_GAIN = 1.0
    CORAL_WRIST_INTEGRAL_GAIN = 0.0
    CORAL_WRIST_DERIVATIVE_GAIN = 0.0

    ALGAE_WRIST_PROPORTIONAL_GAIN = 1.0
    ALGAE_WRIST_INTEGRAL_GAIN = 0.0
    ALGAE_WRIST_DERIVATIVE_GAIN = 0.0

class IntakeState(enum.Enum):
    In = enum.auto
    Out = enum.auto
    Hold = enum.auto

class Intake:
    def __init__(self):
        self.coralWristMotor = rev.SparkMax(0,rev.SparkLowLevel.MotorType.kBrushless)
        self.coralWristEncoder = self.coralWristMotor.getEncoder()

        self.coralMotor = rev.SparkMax(0,rev.SparkLowLevel.MotorType.kBrushless)

        self.algaeWristMotor = rev.SparkMax(0,rev.SparkLowLevel.MotorType.kBrushless)
        self.algaeWristEncoder = self.algaeWristMotor.getEncoder()

        self.rightAlgaeMotor = rev.SparkMax(0,rev.SparkLowLevel.MotorType.kBrushless)
        self.leftAlgaeMotor = rev.SparkMax(0,rev.SparkLowLevel.MotorType.kBrushless)

        self.cooldownTimer = wpilib.Timer()

        self.coralWristController = wpimath.controller.PIDController(
            IntakeConstants.CORAL_WRIST_PROPORTIONAL_GAIN,
            IntakeConstants.CORAL_WRIST_DERIVATIVE_GAIN,
            IntakeConstants.CORAL_WRIST_INTEGRAL_GAIN
        )
        self.coralWristController.set

        self.algaeWristController = wpimath.controller.PIDController(
            IntakeConstants.ALGAE_WRIST_PROPORTIONAL_GAIN,
            IntakeConstants.ALGAE_WRIST_DERIVATIVE_GAIN,
            IntakeConstants.ALGAE_WRIST_INTEGRAL_GAIN
        )

        self.algaeWristTarget = IntakeConstants.ALGAE_WRIST_STARTING_POSITION
        self.coralWristTarget = IntakeConstants.CORAL_WIRST_STARTING_POSITION

        self.algaeIntakeState = IntakeState.Hold
        self.coralIntakeState = IntakeState.Hold

    # returns true if intake/ejection is complete
    def runIntakesTimed(self) -> bool:
        if self.coralIntakeState == IntakeState.In:
            self.runIntakeEject()
            return False


        if self.coralIntakeState != IntakeState.Hold or self.algaeIntakeState != IntakeState.Hold:
            if self.cooldownTimer.isRunning():
                if self.cooldownTimer.get() < IntakeConstants.ACTIVE_TIME:
                    self.runIntakeEject()
                    return False
                else:
                    self.cooldownTimer.stop()
                    self.cooldownTimer.reset()
                    return True
            else:
                self.cooldownTimer.start()
                return False
        else:
            self.coralMotor.stopMotor()
            self.leftAlgaeMotor.stopMotor()
            self.rightAlgeaMotor.stopMotor()
            return True
        

    def runWrists(self):
        algaeAmount = self.algaeWristController.calculate(
            self.algaeWristEncoder.getPosition() * IntakeConstants.ALGAE_ENCODER_ROTATIONS_TO_RADIANS_MULTIPLIER,
            self.algaeWristTarget.radians
        )
        coralAmount = self.coralWristController.calculate(
            self.coralWristEncoder.getPosition() * IntakeConstants.CORAL_ENCODER_ROTATIONS_TO_RADIANS_MULTIPLIER,
            self.coralWristTarget.radians
        )

        self.algaeWristMotor.set(algaeAmount)
        self.coralWristMotor.set(coralAmount)
    def runIntakeEject(self):
        if self.coralIntakeState == IntakeState.In:
            self.coralMotor.set(-IntakeConstants.CORAL_INTAKE_MAX_SPEED)
        if self.coralIntakeState == IntakeState.Out:
            self.coralMotor.set(IntakeConstants.CORAL_INTAKE_MAX_SPEED)
        
        if self.algaeIntakeState == IntakeState.In:
            self.leftAlgaeMotor.set(-IntakeConstants.ALGAE_INTAKE_MAX_SPEED)
            self.rightAlgaeMotor.set(IntakeConstants.ALGAE_INTAKE_MAX_SPEED)
        if self.algaeIntakeState == IntakeState.Out:
            self.leftAlgaeMotor.set(IntakeConstants.ALGAE_INTAKE_MAX_SPEED)
            self.rightAlgaeMotor.set(-IntakeConstants.ALGAE_INTAKE_MAX_SPEED)

    def suspendTask(self):
        self.cooldownTimer.stop()
        self.cooldownTimer.reset()

        self.coralMotor.stopMotor()
        self.leftAlgaeMotor.stopMotor()
        self.rightAlgeaMotor.stopMotor()

    
    def setL1(self):
        self.algaeWristTarget = IntakeConstants.ALGAE_WRIST_STOW_POSITION
        self.coralWristTarget = IntakeConstants.LEVEL_1_CORAL_WRIST_POSITION
        self.algaeIntakeState = IntakeState.Hold
        self.coralIntakeState = IntakeState.Out
        self.suspendTask()

    def setL2(self):
        self.algaeWristTarget = IntakeConstants.ALGAE_WRIST_STOW_POSITION
        self.coralWristTarget = IntakeConstants.LEVEL_MID_CORAL_WRIST_POSITION
        self.algaeIntakeState = IntakeState.Hold
        self.coralIntakeState = IntakeState.Out
        self.suspendTask()
 
    def setL3(self):
        self.algaeWristTarget = IntakeConstants.ALGAE_WRIST_INTAKE_POSITION
        self.coralWristTarget = IntakeConstants.LEVEL_MID_CORAL_WRIST_POSITION
        self.algaeIntakeState = IntakeState.In
        self.coralIntakeState = IntakeState.Out
        self.suspendTask()
 
    def setL4(self):
        self.algaeWristTarget = IntakeConstants.ALGAE_WRIST_STOW_POSITION
        self.coralWristTarget = IntakeConstants.LEVEL_4_CORAL_WRIST_POSITION
        self.algaeIntakeState = IntakeState.Hold
        self.coralIntakeState = IntakeState.Out
        self.suspendTask()
    
    def setHighAlgae(self):
        self.algaeWristTarget = IntakeConstants.ALGAE_WRIST_INTAKE_POSITION
        self.coralWristTarget = IntakeConstants.CORAL_WRIST_STOW_POSITION
        self.algaeIntakeState = IntakeState.In
        self.coralIntakeState = IntakeState.Hold
        self.suspendTask()
    
    def setProcessor(self):
        self.algaeWristTarget = IntakeConstants.ALGAE_WRIST_EJECT_POSITION
        self.coralWristTarget = IntakeConstants.CORAL_WRIST_STOW_POSITION
        self.algaeIntakeState = IntakeState.Out
        self.coralIntakeState = IntakeState.Hold
        self.suspendTask()
 
    def setStation(self):
        self.algaeWristTarget = IntakeConstants.ALGAE_WRIST_STOW_POSITION
        self.coralWristTarget = IntakeConstants.INTAKE_CORAL_WRIST_POSITION
        self.algaeIntakeState = IntakeState.Hold
        self.coralIntakeState = IntakeState.In
        self.suspendTask()
    
    def setIdle(self):
        self.algaeWristTarget = IntakeConstants.ALGAE_WRIST_STOW_POSITION
        self.coralWristTarget = IntakeConstants.CORAL_WRIST_STOW_POSITION
        self.algaeIntakeState = IntakeState.Hold
        self.coralIntakeState = IntakeState.Hold
        self.suspendTask()

 
    
 


