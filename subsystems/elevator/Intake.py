
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
    CORAL_EJECT_SPEED = -0.4
    ALGAE_INTAKE_MAX_SPEED = 0.5

    LEVEL_1_CORAL_WRIST_POSITION = wpimath.geometry.Rotation2d.fromDegrees(0)
    LEVEL_MID_CORAL_WRIST_POSITION = wpimath.geometry.Rotation2d.fromDegrees(-15)
    LEVEL_4_CORAL_WRIST_POSITION = wpimath.geometry.Rotation2d.fromDegrees(0)
    INTAKE_CORAL_WRIST_POSITION = wpimath.geometry.Rotation2d.fromDegrees(35)

    CORAL_WRIST_STARTING_POSITION = wpimath.geometry.Rotation2d.fromDegrees(72)
    CORAL_WRIST_STOW_POSITION = wpimath.geometry.Rotation2d.fromDegrees(60) # stow up

    CORAL_ENCODER_ROTATIONS_TO_DEGREES_MULTIPLIER = 72/-5.2857 # march 22 2025
    CORAL_POSITION_OFFSET = 72 # march something 2025


    ALGAE_PDP_CHANNEL = 11
    CORAL_PDP_CHANNEL = 13
    CORAL_WRIST_PDP_CHANNEL = 12

    ALGAE_IN_CURRENT_THRESHOLD = 25
    CORAL_IN_CURRENT_THRESHOLD = 10
    CORAL_EJECT_CURENT_THRESHOLD = 2

    CORAL_WRIST_STATIC_GAIN = 0.01
    CORAL_WRIST_GRAVITY_GAIN = 0.6 # march 21 2025
    CORAL_WRIST_VELOCITY_GAIN = 0.0
    
    CORAL_WRIST_PROPORTIONAL_GAIN = 8 # march 21 2025
    CORAL_WRIST_INTEGRAL_GAIN = 0.1
    CORAL_WRIST_DERIVATIVE_GAIN = 0.2

    CORAL_WRIST_MAX_VELOCITY_RADIANS_PER_SECOND = math.pi / 4
    CORAL_WRIST_MAX_ACCELERATION_RADIANS_PER_SECOND_SQUARED = math.pi

    CORAL_WRIST_CONSTRAINTS = wpimath.trajectory.TrapezoidProfile.Constraints(
        CORAL_WRIST_MAX_VELOCITY_RADIANS_PER_SECOND,
        CORAL_WRIST_MAX_ACCELERATION_RADIANS_PER_SECOND_SQUARED
    )

    WRIST_MOTOR_CONFIG = rev.SparkBaseConfig().smartCurrentLimit(27)
    INTAKE_MOTOR_CONFIG = rev.SparkBaseConfig().smartCurrentLimit(25)


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

        self.zeroEncoder()

        self.coralMotor = rev.SparkMax(16,rev.SparkLowLevel.MotorType.kBrushless)
        self.coralMotor.configure(
            IntakeConstants.INTAKE_MOTOR_CONFIG,
            rev.SparkBase.ResetMode.kResetSafeParameters,
            rev.SparkBase.PersistMode.kPersistParameters
        )

        # self.rightAlgaeMotor = rev.SparkMax(14,rev.SparkLowLevel.MotorType.kBrushless)
        # self.rightAlgaeMotor.configure(
        #     IntakeConstants.INTAKE_MOTOR_CONFIG,
        #     rev.SparkBase.ResetMode.kResetSafeParameters,
        #     rev.SparkBase.PersistMode.kPersistParameters
        # )
        
        self.leftAlgaeMotor = rev.SparkMax(13,rev.SparkLowLevel.MotorType.kBrushless)
        self.leftAlgaeMotor.configure(
            IntakeConstants.INTAKE_MOTOR_CONFIG,
            rev.SparkBase.ResetMode.kResetSafeParameters,
            rev.SparkBase.PersistMode.kPersistParameters
        )

        # self.cooldownTimer = wpilib.Timer()

        self.coralWristFeedForeward = wpimath.controller.ArmFeedforward(
            IntakeConstants.CORAL_WRIST_STATIC_GAIN,
            IntakeConstants.CORAL_WRIST_GRAVITY_GAIN,
            IntakeConstants.CORAL_WRIST_VELOCITY_GAIN
        )

        self.coralWristController = wpimath.controller.ProfiledPIDController( # uses radians
            IntakeConstants.CORAL_WRIST_PROPORTIONAL_GAIN,
            IntakeConstants.CORAL_WRIST_INTEGRAL_GAIN,
            IntakeConstants.CORAL_WRIST_DERIVATIVE_GAIN,
            IntakeConstants.CORAL_WRIST_CONSTRAINTS
        )

        self.coralWristController.enableContinuousInput(0,360)

        self.coralWristController.setTolerance(0.1) # radians

        self.coralWristController.reset(self.getWristPosition().radians())

        self.coralWristTarget = IntakeConstants.CORAL_WRIST_STOW_POSITION

        # these are to record algae currents to know if we have been above threshold for 0.1 seconds (0.02s per cycle * 5 cycles)
        self.previousAlgaeCurrents = [0.0, 0.0, 0.0, 0.0, 0.0]
        self.previousCoralCurrents = [0.0, 0.0, 0.0, 0.0, 0.0]

        self.intaking = False

        self.objectInTestingVariable = False # variables for testing
        self.objectOutTestingVariable = True
        
        self.setIdle()


    
    def zeroEncoder(self):
        self.coralWristEncoder.setPosition(0.0)
        
    def getWristPosition(self) -> wpimath.geometry.Rotation2d:
        position = self.coralWristEncoder.getPosition() * IntakeConstants.CORAL_ENCODER_ROTATIONS_TO_DEGREES_MULTIPLIER + IntakeConstants.CORAL_POSITION_OFFSET
        positionRadians = wpimath.angleModulus(wpimath.units.degreesToRadians(position))
        return wpimath.geometry.Rotation2d(positionRadians)

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

    def updateCurrentDrawHistory(self): # current as in amps
        # pop the back and push to the front
        # remove the oldest one and add a new measurement
        self.previousAlgaeCurrents.pop(0)
        self.previousAlgaeCurrents.append(self.pdpReference.getCurrent(IntakeConstants.ALGAE_PDP_CHANNEL))

        self.previousCoralCurrents.pop(0)
        self.previousCoralCurrents.append(self.pdpReference.getCurrent(IntakeConstants.CORAL_PDP_CHANNEL))


    def algaeAllTheWayIn(self) -> bool:
        return False

        #return self.objectInTestingVariable

        # for current in self.previousAlgaeCurrents:
        #     if current < IntakeConstants.ALGAE_IN_CURRENT_THRESHOLD:
        #         return False
        # return True
    
    def algaeAllTheWayOut(self) -> bool:
        return False
        # return self.objectOutTestingVariable
        # return False # todo
    

    def coralAllTheWayIn(self) -> bool:
        return self.objectInTestingVariable

        # for current in self.previousCoralCurrents:
        #     if current < IntakeConstants.CORAL_IN_CURRENT_THRESHOLD:
        #         return False
        # return True
    
    def coralAllTheWayOut(self) -> bool:
        return self.objectOutTestingVariable
        # return False # todo
    
    def runWrist(self):
        #return # early return for testing the elevator state machine
        wristPosition = self.getWristPosition()
        pidAmount = self.coralWristController.calculate(
            self.getWristPosition().radians(),
            self.coralWristTarget.radians()
        ) # this output will now be in volts

        feedforward = self.coralWristFeedForeward.calculate(
            wristPosition.radians(),
            wpimath.units.rotationsPerMinuteToRadiansPerSecond(self.coralWristEncoder.getVelocity())
        )

        output = pidAmount + feedforward
        
        if output > 12.0:
            output = 12.0
        if output < -12.0:
            output = 12.0

        self.coralWristMotor.setVoltage(-output)

    def setTargetAngle(self,targetAngleDegrees: float):
        self.coralWristTarget = wpimath.geometry.Rotation2d.fromDegrees(targetAngleDegrees)

    def runIntakeEject(self, active: bool):
        if active:
            if not self.intaking:
                self.coralMotor.set(-0.3) # intake
                self.leftAlgaeMotor.set(0.0)
            else:
                self.coralMotor.set(0.6) # eject
                self.leftAlgaeMotor.set(-0.5)
        else:
            self.coralMotor.set(0.0)
            self.leftAlgaeMotor.set(0.0)

        
    def runWristRaw(self,amount: float):
        self.coralWristMotor.set(amount)
    
    # def runAlgaeRaw(self,amount: float):
    #     self.leftAlgaeMotor.set(amount)
    #     # self.rightAlgaeMotor.set(-amount)

    # def runIntakeEject(self):
    #     if self.coralIntakeState.name == IntakeState.In.name:
    #         self.coralMotor.set(-IntakeConstants.CORAL_INTAKE_SPEED)
    #     elif self.coralIntakeState.name == IntakeState.Out.name:
    #         self.coralMotor.set(IntakeConstants.CORAL_EJECT_SPEED)
    #     elif self.coralIntakeState.name == IntakeState.Hold.name:
    #         self.coralMotor.set(0.0)
    #     else:
    #         self.coralMotor.set(0.0)

    #     if self.algaeIntakeState.name == IntakeState.In.name:
    #         self.leftAlgaeMotor.set(-IntakeConstants.ALGAE_INTAKE_MAX_SPEED)
    #     elif self.algaeIntakeState.name == IntakeState.Out.name:
    #         self.leftAlgaeMotor.set(IntakeConstants.ALGAE_INTAKE_MAX_SPEED)
    #     elif self.algaeIntakeState.name == IntakeState.Hold.name:
    #         self.leftAlgaeMotor.set(0.0)
    #     else:
    #         self.leftAlgaeMotor.set(0.0)
            

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
        SmartDashboard.putNumber("target wrist position", self.coralWristTarget.degrees())

    def logWristSafety(self):
        SmartDashboard.putNumber("wrist current",self.coralWristMotor.getOutputCurrent())
        SmartDashboard.putNumber("wrist temperature",self.coralWristMotor.getMotorTemperature())



    def setL1(self):
        self.coralWristTarget = IntakeConstants.LEVEL_1_CORAL_WRIST_POSITION
        self.intaking = False

    def setL2(self):
        self.coralWristTarget = IntakeConstants.LEVEL_MID_CORAL_WRIST_POSITION
        self.intaking = False
 
    def setL3(self): # coral
        self.coralWristTarget = IntakeConstants.LEVEL_MID_CORAL_WRIST_POSITION
        self.intaking = False
 
    def setL4(self):
        self.coralWristTarget = IntakeConstants.LEVEL_4_CORAL_WRIST_POSITION
        self.intaking = False
    
    def setAlgae(self):
        self.coralWristTarget = IntakeConstants.CORAL_WRIST_STOW_POSITION
        self.intaking = True
    
    def setProcessor(self):
        self.coralWristTarget = IntakeConstants.CORAL_WRIST_STOW_POSITION
        self.intaking = False
 
    def setStation(self):
        self.coralWristTarget = IntakeConstants.INTAKE_CORAL_WRIST_POSITION
        self.intaking = True
    
    def setIdle(self):
        self.coralWristTarget = IntakeConstants.CORAL_WRIST_STOW_POSITION
        self.intaking = True