#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

import math
import wpilib
import wpimath.kinematics
import wpimath.geometry
import wpimath.controller
import wpimath.trajectory

import rev._rev as rev
from phoenix6 import hardware as ctre

kWheelRadius = 0.0508
kEncoderResolution = 4096
kModuleMaxAngularVelocity = math.pi
kModuleMaxAngularAcceleration = math.tau


class ModuleConstants:
    WHEEL_RADIUS = 0.0

    # assume all values are untuned unless specified with a date of tuning
    MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = 0.0
    MAX_ANGULAR_ACCELERATION_RADIANS_PER_SECOND_SQUARED = 0.0
    TURN_PROPORTIONAL_GAIN = 1.0
    TURN_INTEGRAL_GAIN = 0.0
    TURN_DERIVATIVE_GAIN = 0.0
    TURN_STATIC_GAIN_VOLTS = 1.0
    TURN_VELOCITY_GAIN_VOLT_SECONDS_PER_RADIAN = 0.5

    # assume all values are untuned unless specified with a date of tuning
    MAX_VELOCITY_METERS_PER_SECOND = 0.0
    MAX_ACCELERATION_METERS_PER_SECOND_SQUARED = 0.0
    DRIVE_PROPORTIONAL_GAIN = 1.0
    DRIVE_INTEGRAL_GAIN = 0.0
    DRIVE_DERIVATIVE_GAIN = 0.0
    DRIVE_STATIC_GAIN_VOLTS = 1.0
    DRIVE_VELOCITY_GAIN_VOLT_SECONDS_PER_METER = 0.5

    

class SwerveModule:
    def __init__(
        self,
        driveMotorCANID: int,
        turningMotorCANID: int,
        turningEncoderCANID: int
    ) -> None:
        """Constructs a SwerveModule with a drive motor, turning motor, drive encoder and turning encoder.

        :param driveMotorCANID:      CANID of drive motor
        :param turningMotorCANID:    CANID of turn motor
        :param turningEncoderCANID:  CANID of the absolute encoder on the module
        """
        self.driveMotor = wpilib.CANSparkMax(driveMotorCANID,rev.CANSparkLowLevel.kBrushless)
        self.turningMotor = wpilib.PWMSparkMax(turningMotorCANID,rev.CANSparkLowLevel.kBrushless)

        self.driveEncoder = self.driveMotor.getEncoder()
        self.turningEncoder = self.turningMotor.getEncoder()

        self.moduleEncoder = ctre.CANcoder(turningEncoderCANID)

        # Gains are for example purposes only - must be determined for your own robot!
        self.drivePIDController = wpimath.controller.PIDController(
            ModuleConstants.DRIVE_PROPORTIONAL_GAIN,
            ModuleConstants.DRIVE_INTEGRAL_GAIN,
            ModuleConstants.DRIVE_DERIVATIVE_GAIN,
        )
        # Gains are for example purposes only - must be determined for your own robot!
        self.turningPIDController = wpimath.controller.PIDController(
            ModuleConstants.TURN_PROPORTIONAL_GAIN,
            ModuleConstants.TURN_INTEGRAL_GAIN,
            ModuleConstants.DRIVE_DERIVATIVE_GAIN
        )
        # Gains are for example purposes only - must be determined for your own robot!
        self.driveFeedforward = wpimath.controller.SimpleMotorFeedforwardMeters(
            ModuleConstants.DRIVE_STATIC_GAIN_VOLTS,
            ModuleConstants.DRIVE_VELOCITY_GAIN_VOLT_SECONDS_PER_METER
        )
        self.turnFeedforward = wpimath.controller.SimpleMotorFeedforwardRadians(
            ModuleConstants.TURN_STATIC_GAIN_VOLTS,
            ModuleConstants.TURN_VELOCITY_GAIN_VOLT_SECONDS_PER_RADIAN
        )

        # Set the distance per pulse for the drive encoder. We can simply use the
        # distance traveled for one rotation of the wheel divided by the encoder
        # resolution.
        self.driveEncoder.setDistancePerPulse(
            math.tau * kWheelRadius / kEncoderResolution
        )

        # Set the distance (in this case, angle) in radians per pulse for the turning encoder.
        # This is the the angle through an entire rotation (2 * pi) divided by the
        # encoder resolution.
        self.turningEncoder.setDistancePerPulse(math.tau / kEncoderResolution)
        
        # Limit the PID Controller's input range between -pi and pi and set the input
        # to be continuous.
        self.turningPIDController.enableContinuousInput(-math.pi, math.pi)

    def getState(self) -> wpimath.kinematics.SwerveModuleState:
        """Returns the current state of the module.

        :returns: The current state of the module.
        """
        return wpimath.kinematics.SwerveModuleState(
            self.driveEncoder.getRate(),
            wpimath.geometry.Rotation2d(self.turningEncoder.getDistance()),
        )

    def getPosition(self) -> wpimath.kinematics.SwerveModulePosition:
        """Returns the current position of the module.

        :returns: The current position of the module.
        """
        return wpimath.kinematics.SwerveModulePosition(
            self.driveEncoder.getDistance(),
            wpimath.geometry.Rotation2d(self.turningEncoder.getDistance()),
        )

    def setDesiredState(
        self, desiredState: wpimath.kinematics.SwerveModuleState
    ) -> None:
        """Sets the desired state for the module.

        :param desiredState: Desired state with speed and angle.
        """

        encoderRotation = wpimath.geometry.Rotation2d(self.turningEncoder.getDistance())

        # Optimize the reference state to avoid spinning further than 90 degrees
        state = wpimath.kinematics.SwerveModuleState.optimize(
            desiredState, encoderRotation
        )

        # Scale speed by cosine of angle error. This scales down movement perpendicular to the desired
        # direction of travel that can occur when modules change directions. This results in smoother
        # driving.
        state.speed *= (state.angle - encoderRotation).cos()

        # Calculate the drive output from the drive PID controller.
        driveOutput = self.drivePIDController.calculate(
            self.driveEncoder.getRate(), state.speed
        )

        driveFeedforward = self.driveFeedforward.calculate(state.speed)

        # Calculate the turning motor output from the turning PID controller.
        turnOutput = self.turningPIDController.calculate(
            self.turningEncoder.getDistance(), state.angle.radians()
        )

        turnFeedforward = self.turnFeedforward.calculate(
            self.turningPIDController.getSetpoint().velocity
        )

        self.driveMotor.setVoltage(driveOutput + driveFeedforward)
        self.turningMotor.setVoltage(turnOutput + turnFeedforward)