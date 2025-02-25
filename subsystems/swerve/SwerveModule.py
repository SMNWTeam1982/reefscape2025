#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

import math
import wpimath.kinematics
import wpimath.geometry
import wpimath.controller
import wpimath.trajectory

import rev
from phoenix6 import hardware as ctre

class ModuleConstants:
    # value taken from 2024 code
    RPM_TO_METERS_PER_SECOND_CONVERSION_MULTIPLIER = 7.049382716E-4
    POSITION_TO_METERS_TRAVELED_MULTIPLIER = 0.2855

    # assume all values are untuned unless specified with a date of tuning
    TURN_PROPORTIONAL_GAIN = 0.73 *1 # Jan 18 2025
    TURN_INTEGRAL_GAIN = 0.0 # Jan 18 2025
    TURN_DERIVATIVE_GAIN = 0.01 *1 # Jan 18 2025

    # assume all values are untuned unless specified with a date of tuning
    DRIVE_PROPORTIONAL_GAIN = 0.0
    DRIVE_INTEGRAL_GAIN = 0.0
    DRIVE_DERIVATIVE_GAIN = 0.0
    DRIVE_STATIC_GAIN_VOLTS = 0.05 # this number is made up
    DRIVE_VELOCITY_GAIN_VOLT_SECONDS_PER_METER = 2.87 # Jan 18 2025

    

class Wheel:
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
        self.driveMotor = rev.SparkMax(driveMotorCANID,rev.SparkLowLevel.MotorType.kBrushless)
        self.turningMotor = rev.SparkMax(turningMotorCANID,rev.SparkLowLevel.MotorType.kBrushless)

        # this encoder measures wheel speed and distance traveled
        self.driveEncoder = self.driveMotor.getEncoder()

        self.moduleEncoder = ctre.CANcoder(turningEncoderCANID) # this encoder measures wheel direcion

        # these are for increasing or decreasing wheel speed to match desired speed
        self.drivePIDController = wpimath.controller.PIDController( 
            ModuleConstants.DRIVE_PROPORTIONAL_GAIN, # should only need this one others can be 0
            ModuleConstants.DRIVE_INTEGRAL_GAIN,
            ModuleConstants.DRIVE_DERIVATIVE_GAIN,
        ) # this controller outputs an amount to change the voltage by, this doesn't control the motor

        # get the wheel to snap quickly to where you want
        self.turningPIDController = wpimath.controller.PIDController(
            ModuleConstants.TURN_PROPORTIONAL_GAIN, # most useful one
            ModuleConstants.TURN_INTEGRAL_GAIN, # shouldnt be any constant error, leave to 0
            ModuleConstants.TURN_DERIVATIVE_GAIN # tune only with small changes at a time
        )
        
        self.driveFeedforward = wpimath.controller.SimpleMotorFeedforwardMeters(
            ModuleConstants.DRIVE_STATIC_GAIN_VOLTS, # minimum voltage needed to move
            
            # voltage/velocity, get average between different constant voltages and velocity measurements
            ModuleConstants.DRIVE_VELOCITY_GAIN_VOLT_SECONDS_PER_METER
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
            wpimath.geometry.Rotation2d.fromRotations(self.moduleEncoder.get_position().value),
        )

    def getPosition(self) -> wpimath.kinematics.SwerveModulePosition:
        """Returns the current position of the module.

        :returns: The current position of the module.
        """
        return wpimath.kinematics.SwerveModulePosition(
            self.driveEncoder.getPosition() * ModuleConstants.POSITION_TO_METERS_TRAVELED_MULTIPLIER,
            wpimath.geometry.Rotation2d.fromRotations(self.moduleEncoder.get_position().value),
        )

    def setDesiredState(
        self, desiredState: wpimath.kinematics.SwerveModuleState
    ) -> None:
        """Sets the desired state for the module.

        :param desiredState: Desired state with speed (m/s) and angle (Rotation2D)
        """

        encoderRotation = wpimath.geometry.Rotation2d.fromRotations(self.moduleEncoder.get_position().value)

        # Optimize the reference state to avoid spinning further than 90 degrees
        desiredState.optimize(encoderRotation)

        # Scale speed by cosine of angle error. This scales down movement perpendicular to the desired
        # direction of travel that can occur when modules change directions. This results in smoother
        # driving.

        desiredState.cosineScale(encoderRotation)

        # Calculate the drive output from the drive PID controller. this will be added to the FF voltage
        driveOutput = self.drivePIDController.calculate(
            self.driveEncoder.getVelocity(), desiredState.speed
        )

        driveFeedforward = self.driveFeedforward.calculate(desiredState.speed) # converts mps to volts

        # Calculate the turning motor output from the turning PID controller.
        turnOutput = self.turningPIDController.calculate(
            encoderRotation.radians(), desiredState.angle.radians()
        )
        
        if turnOutput > 1.0:
            turnOutput = 1.0
        if turnOutput < -1.0:
            turnOutput = -1.0

        self.driveMotor.setVoltage( driveOutput + driveFeedforward ) # Volts because of feedforward

        self.turningMotor.set(-turnOutput) # use percent for turning

    def updateTurnPID(self, p: float, i: float, d: float):
        self.turningPIDController.setPID(p,i,d)
    
    def updateDrivePID(self, p: float, i: float, d: float):
        self.drivePIDController.setPID(p,i,d)