
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

    CORAL_INTAKE_SPEED = 0.6
    CORAL_EJECT_SPEED = -0.2
    ALGAE_INTAKE_MAX_SPEED = 0.5

    LEVEL_1_CORAL_WRIST_POSITION = wpimath.geometry.Rotation2d.fromDegrees(0)
    LEVEL_MID_CORAL_WRIST_POSITION = wpimath.geometry.Rotation2d.fromDegrees(-35)
    LEVEL_4_CORAL_WRIST_POSITION = wpimath.geometry.Rotation2d.fromDegrees(-10)
    INTAKE_CORAL_WRIST_POSITION = wpimath.geometry.Rotation2d.fromDegrees(35)

    CORAL_WRIST_STARTING_POSITION = wpimath.geometry.Rotation2d.fromDegrees(72)
    CORAL_WRIST_STOW_POSITION = wpimath.geometry.Rotation2d.fromDegrees(50) # stow up

    CORAL_ENCODER_ROTATIONS_TO_DEGREES_MULTIPLIER = (1/3.666663) * -133 # march 8 2025
    CORAL_POSITION_OFFSET = 72


    ALGAE_PDP_CHANNEL = 11
    CORAL_PDP_CHANNEL = 13
    CORAL_WRIST_PDP_CHANNEL = 12
    ALGAE_IN_CURRENT_THRESHOLD = 25
    CORAL_IN_CURRENT_THRESHOLD = 8
    CORAL_EJECT_CURENT_THRESHOLD = 2


    CORAL_WRIST_PROPORTIONAL_GAIN = 0.007
    CORAL_WRIST_INTEGRAL_GAIN = 0.002
    CORAL_WRIST_DERIVATIVE_GAIN = 0.0002
    
    CORAL_WRIST_OUTPUT_LIMIT = 0.4

    WRIST_MOTOR_CONFIG = rev.SparkBaseConfig().smartCurrentLimit(10,15,5000)
    INTAKE_MOTOR_CONFIG = rev.SparkBaseConfig().smartCurrentLimit(10,15,8000)


class IntakeState(enum.Enum):
    In = enum.auto
    Out = enum.auto
    Hold = enum.auto

class Intake:
    def __init__(self, pdpReference: wpilib.PowerDistribution):
        self.pdpReference = pdpReference

        self.coralWristMotor = rev.SparkMax(15,rev.SparkLowLevel.MotorType.kBrushless)
        self.coralWristEncoder = self.coralWristMotor.getEncoder()
        self.coralWristMotor.configure(
            IntakeConstants.WRIST_MOTOR_CONFIG,
            rev.SparkBase.ResetMode.kResetSafeParameters,
            rev.SparkBase.PersistMode.kPersistParameters
        )

        self.coralMotor = rev.SparkMax(16,rev.SparkLowLevel.MotorType.kBrushless)
        self.coralMotor.configure(
            IntakeConstants.INTAKE_MOTOR_CONFIG,
            rev.SparkBase.ResetMode.kResetSafeParameters,
            rev.SparkBase.PersistMode.kPersistParameters
        )

        self.rightAlgaeMotor = rev.SparkMax(14,rev.SparkLowLevel.MotorType.kBrushless)
        self.rightAlgaeMotor.configure(
            IntakeConstants.INTAKE_MOTOR_CONFIG,
            rev.SparkBase.ResetMode.kResetSafeParameters,
            rev.SparkBase.PersistMode.kPersistParameters
        )
        self.leftAlgaeMotor = rev.SparkMax(13,rev.SparkLowLevel.MotorType.kBrushless)
        self.leftAlgaeMotor.configure(
            IntakeConstants.INTAKE_MOTOR_CONFIG,
            rev.SparkBase.ResetMode.kResetSafeParameters,
            rev.SparkBase.PersistMode.kPersistParameters
        )

        # self.cooldownTimer = wpilib.Timer()

        self.coralWristController = wpimath.controller.PIDController(
            IntakeConstants.CORAL_WRIST_PROPORTIONAL_GAIN,
            IntakeConstants.CORAL_WRIST_INTEGRAL_GAIN,
            IntakeConstants.CORAL_WRIST_DERIVATIVE_GAIN
        )
        
        self.coralWristController.setTolerance(1) # degree
        self.coralWristTarget = IntakeConstants.CORAL_WRIST_STOW_POSITION

        self.algaeIntakeState = IntakeState.Hold
        self.coralIntakeState = IntakeState.Hold
    
    def zeroEncoder(self):
        self.coralWristEncoder.setPosition(0.0)
    def getWristPosition(self) -> wpimath.geometry.Rotation2d:
        position = self.coralWristEncoder.getPosition() * IntakeConstants.CORAL_ENCODER_ROTATIONS_TO_DEGREES_MULTIPLIER
        return wpimath.geometry.Rotation2d.fromDegrees(position + IntakeConstants.CORAL_POSITION_OFFSET)

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
    def algaeAllTheWayOut(self) -> bool:
        return False # todo
    
    def coralAllTheWayIn(self) -> bool:
        return self.pdpReference.getCurrent(IntakeConstants.CORAL_PDP_CHANNEL) > IntakeConstants.CORAL_IN_CURRENT_THRESHOLD
    def coralAllTheWayOut(self) -> bool:
        return False # todo
    
    def runWrist(self):
        coralAmount = self.coralWristController.calculate(
            self.getWristPosition().degrees(),
            self.coralWristTarget.degrees()
        )
        
        if abs(coralAmount) > IntakeConstants.CORAL_WRIST_OUTPUT_LIMIT:
            if coralAmount > 0:
                coralAmount = IntakeConstants.CORAL_WRIST_OUTPUT_LIMIT
            else:
                coralAmount = -IntakeConstants.CORAL_WRIST_OUTPUT_LIMIT

        self.coralWristMotor.set(-coralAmount)

    def runIntakeEject(self) -> bool:
        if self.coralIntakeState == IntakeState.In:
            self.coralMotor.set(-IntakeConstants.CORAL_INTAKE_SPEED)
            # if self.coralAllTheWayIn(): state will be controlled from the elevator state machines
            #     self.setIdle() 
        if self.coralIntakeState == IntakeState.Out:
            self.coralMotor.set(IntakeConstants.CORAL_EJECT_SPEED)
        if self.coralIntakeState == IntakeState.Hold:
            self.coralMotor.set(0.0)

        if self.algaeIntakeState == IntakeState.In:
            self.leftAlgaeMotor.set(-IntakeConstants.ALGAE_INTAKE_MAX_SPEED)
            self.rightAlgaeMotor.set(IntakeConstants.ALGAE_INTAKE_MAX_SPEED)
            # if self.algaeAllTheWayIn():
            #     self.setIdle()
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
        SmartDashboard.putNumber("coralWristPosition", self.getWristPosition().degrees())
        SmartDashboard.putNumber("raw coralWristPosition", self.coralWristEncoder.getPosition())

    def logPIDErrors(self):
        SmartDashboard.putNumber("wrist p error", self.coralWristController.getError())
        SmartDashboard.putNumber("wrist i error", self.coralWristController.getAccumulatedError())
        SmartDashboard.putNumber("wrist d error", self.coralWristController.getErrorDerivative())
        
    def logMotorStats(self):
        SmartDashboard.putNumber("wrist current",self.coralWristMotor.getOutputCurrent())
        SmartDashboard.putNumber("wrist temperature",self.coralWristMotor.getMotorTemperature())
        
        


    
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
 
    def setL3(self): # coral
        self.coralWristTarget = IntakeConstants.LEVEL_MID_CORAL_WRIST_POSITION
        self.algaeIntakeState = IntakeState.Hold
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