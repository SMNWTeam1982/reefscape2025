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

class ModuleConstants:
    WHEEL_RADIUS = 0.0

    # value taken from 2024 code
    RPM_TO_METERS_PER_SECOND_CONVERSION_MULTIPLIER = 7.049382716E-4
    POSITION_TO_METERS_TRAVELED_MULTIPLIER = 0.2855

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
        self.driveMotor = rev.CANSparkMax(driveMotorCANID,rev.CANSparkLowLevel.kBrushless)
        self.turningMotor = rev.CANSparkMax(turningMotorCANID,rev.CANSparkLowLevel.kBrushless)

        # this encoder measures wheel speed and distance traveled
        self.driveEncoder = self.driveMotor.getEncoder()

        self.moduleEncoder = ctre.CANcoder(turningEncoderCANID) # this encoder measures wheel direcion

        # these are for increasing or decreasing wheel speed to match desired speed
        self.drivePIDController = wpimath.controller.PIDController( 
            ModuleConstants.DRIVE_PROPORTIONAL_GAIN, # should only need this one others can be 0
            ModuleConstants.DRIVE_INTEGRAL_GAIN,
            ModuleConstants.DRIVE_DERIVATIVE_GAIN,
        )

        # get the wheel to snap quickly to where you want
        self.turningPIDController = wpimath.controller.PIDController(
            ModuleConstants.TURN_PROPORTIONAL_GAIN, # most useful one
            ModuleConstants.TURN_INTEGRAL_GAIN, # shouldnt be any constant error, leave to 0
            ModuleConstants.DRIVE_DERIVATIVE_GAIN # tune only with small changes at a time
        )
        
        # idk what these do, something to do with speed - zach
        self.driveFeedforward = wpimath.controller.SimpleMotorFeedforwardMeters(
            ModuleConstants.DRIVE_STATIC_GAIN_VOLTS,
            ModuleConstants.DRIVE_VELOCITY_GAIN_VOLT_SECONDS_PER_METER
        )
        self.turnFeedforward = wpimath.controller.SimpleMotorFeedforwardRadians(
            ModuleConstants.TURN_STATIC_GAIN_VOLTS,
            ModuleConstants.TURN_VELOCITY_GAIN_VOLT_SECONDS_PER_RADIAN
        )
        
        # Limit the PID Controller's input range between -pi and pi and set the input
        # to be continuous.
        self.turningPIDController.enableContinuousInput(-math.pi, math.pi) # moves in a circle

    def getState(self) -> wpimath.kinematics.SwerveModuleState:
        """Returns the current state of the module.

        :returns: The current state of the module.
        """
        return wpimath.kinematics.SwerveModuleState(
            self.driveEncoder.getVelocity() * ModuleConstants.RPM_TO_METERS_PER_SECOND_CONVERSION_MULTIPLIER,
            wpimath.geometry.Rotation2d.fromRotations(self.moduleEncoder.get_position()),
        )

    def getPosition(self) -> wpimath.kinematics.SwerveModulePosition:
        """Returns the current position of the module.

        :returns: The current position of the module.
        """
        return wpimath.kinematics.SwerveModulePosition(
            self.driveEncoder.getDistance() * ModuleConstants.POSITION_TO_METERS_TRAVELED_MULTIPLIER,
            wpimath.geometry.Rotation2d.fromRotations(self.moduleEncoder.get_position()),
        )

    def setDesiredState(
        self, desiredState: wpimath.kinematics.SwerveModuleState
    ) -> None:
        """Sets the desired state for the module.

        :param desiredState: Desired state with speed (m/s) and angle (Rotation2D)
        """

        encoderRotation = wpimath.geometry.Rotation2d.fromRotations(self.moduleEncoder.get_position())

        # Optimize the reference state to avoid spinning further than 90 degrees
        state = wpimath.kinematics.SwerveModuleState.optimize(
            desiredState, encoderRotation
        )

        # Scale speed by cosine of angle error. This scales down movement perpendicular to the desired
        # direction of travel that can occur when modules change directions. This results in smoother
        # driving.
        # this equation returns the cos of the angle between state.angle and encoderRotation
        state.speed *= state.angle.cos() * encoderRotation.cos() + state.angle.sin() * encoderRotation.sin() 

        # Calculate the drive output from the drive PID controller. this will be added to the FF voltage
        driveOutput = self.drivePIDController.calculate(
            self.driveEncoder.getRate(), state.speed
        )

        driveFeedforward = self.driveFeedforward.calculate(state.speed)

        # Calculate the turning motor output from the turning PID controller.
        turnOutput = self.turningPIDController.calculate(
            encoderRotation.radians(), state.angle.radians()
        )

        turnFeedforward = self.turnFeedforward.calculate(
            self.turningPIDController.getSetpoint().velocity # get the motor to move
        )

        self.driveMotor.setVoltage(driveOutput + driveFeedforward) # both of these are in Volts
        self.turningMotor.setVoltage(turnOutput + turnFeedforward) # both of these are in Volts
