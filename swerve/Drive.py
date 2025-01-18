#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#
import wpimath.units
import math
import wpilib
import wpimath.geometry
import wpimath.kinematics
from . import SwerveModule
from phoenix6 import hardware as ctre

from wpilib import SmartDashboard

kMaxSpeed = 3.0  # 3 meters per second
kMaxAngularSpeed = math.pi  # 1/2 rotation per second

class DriveConstants:
    MAX_SPEED_METERS_PER_SECOND = 3.0 # decided on three by converting 4000rpm to mps
    
    # translation values taken from 2024 code
    FRONT_LEFT_LOCATION = wpimath.geometry.Translation2d(0.2635, 0.2635)
    FRONT_RIGHT_LOCATION = wpimath.geometry.Translation2d(0.2635, -0.2635)
    BACK_LEFT_LOCATION = wpimath.geometry.Translation2d(-0.2635, 0.2635)
    BACK_RIGHT_LOCATION = wpimath.geometry.Translation2d(-0.2635, -0.2635)

class Drivetrain:
    """
    Represents a swerve drive style drivetrain.
    """

    def __init__(self) -> None:
        self.frontLeft = SwerveModule.Wheel(3,4,2)
        self.frontRight = SwerveModule.Wheel(5,6,1) # copied from 2024
        self.backLeft = SwerveModule.Wheel(1,2,3)
        self.backRight = SwerveModule.Wheel(7,8,4)
        
        self.gyro = ctre.pigeon2.Pigeon2(0) # copied from 2024

        # unsure if kinematics is constant so I keep here - zach
        # took me far too long to figure you were using the wrong constants - kay
        self.kinematics = wpimath.kinematics.SwerveDrive4Kinematics(
            DriveConstants.FRONT_LEFT_LOCATION,
            DriveConstants.FRONT_RIGHT_LOCATION,
            DriveConstants.BACK_LEFT_LOCATION,
            DriveConstants.BACK_RIGHT_LOCATION,
        )

        self.odometry = wpimath.kinematics.SwerveDrive4Odometry(
            self.kinematics,
            wpimath.geometry.Rotation2d.fromDegrees(self.gyro.get_yaw().value),
            (
                self.frontLeft.getPosition(),
                self.frontRight.getPosition(),
                self.backLeft.getPosition(),
                self.backRight.getPosition(),
            ),
        )

        self.gyro.set_yaw(0)

    def drive(
        self,
        xSpeed: float,
        ySpeed: float,
        rotation: float,
        periodSeconds: float
    ) -> None:
        """
        Method to drive the robot using joystick info.
        :param xSpeed: Speed of the robot in the x direction (forward).
        :param ySpeed: Speed of the robot in the y direction (sideways).
        :param rot: Angular rate of the robot.
        """
        swerveModuleStates = self.kinematics.toSwerveModuleStates(
            wpimath.kinematics.ChassisSpeeds.discretize(
                (
                    wpimath.kinematics.ChassisSpeeds.fromFieldRelativeSpeeds(
                        xSpeed, ySpeed, rotation, wpimath.geometry.Rotation2d.fromDegrees(self.gyro.get_yaw().value)
                    )
                ),
                periodSeconds,
            )
        )
        
        wpimath.kinematics.SwerveDrive4Kinematics.desaturateWheelSpeeds(
            swerveModuleStates, DriveConstants.MAX_SPEED_METERS_PER_SECOND
        )
        
        self.frontLeft.setDesiredState(swerveModuleStates[0])
        self.frontRight.setDesiredState(swerveModuleStates[1])
        self.backLeft.setDesiredState(swerveModuleStates[2])
        self.backRight.setDesiredState(swerveModuleStates[3])

    def updateOdometry(self) -> None:
        """Updates the field relative position of the robot."""
        self.odometry.update(
            wpimath.geometry.Rotation2d.fromDegrees(self.gyro.get_yaw().value),
            (
                self.frontLeft.getPosition(),
                self.frontRight.getPosition(),
                self.backLeft.getPosition(),
                self.backRight.getPosition(),
            ),
        )
    
    def displayTelemetry(self) -> None:
        SmartDashboard.putNumber("gyro angle",self.gyro.get_yaw().value)

        SmartDashboard.putNumber("front left angle error",self.frontLeft.turningPIDController.getPositionError())# self.frontLeft.getPosition().angle.degrees())
        #SmartDashboard.putNumber("front left velocity error",self.frontLeft.drivePIDController.getPositionError())
        
        SmartDashboard.putNumber("front right angle error",self.frontRight.turningPIDController.getPositionError())# self.frontRight.getPosition().angle.degrees())
        #SmartDashboard.putNumber("front right velocity error",self.frontRight.drivePIDController.getPositionError())
        
        SmartDashboard.putNumber("back left angle error",self.backLeft.turningPIDController.getPositionError())# self.backLeft.getPosition().angle.degrees())
        #SmartDashboard.putNumber("back left velocity error",self.backLeft.drivePIDController.getPositionError())
        
        SmartDashboard.putNumber("back right angle error",self.backRight.turningPIDController.getPositionError())# self.backRight.getPosition().angle.degrees())
        #SmartDashboard.putNumber("back right velocity error",self.backRight.drivePIDController.getPositionError())

        SmartDashboard.putNumberArray(
            "velocity errors",
            [
                self.frontLeft.drivePIDController.getPositionError(),
                self.frontRight.drivePIDController.getPositionError(),
                self.backLeft.drivePIDController.getPositionError(),
                self.backRight.drivePIDController.getPositionError()
            ]
        )
        
        SmartDashboard.putNumberArray(
            "velocities",
            [
                self.frontLeft.getState().speed,
                self.frontRight.getState().speed,
                self.backLeft.getState().speed,
                self.backRight.getState().speed
            ]
        )

    def displayTurnPID(self):
        SmartDashboard.putNumber("p",self.frontLeft.turningPIDController.getP())
        SmartDashboard.putNumber("i",self.frontLeft.turningPIDController.getI())
        SmartDashboard.putNumber("d",self.frontLeft.turningPIDController.getD())
    
    def updateTurnPIDs(self):
        p = SmartDashboard.getNumber("p",self.frontLeft.turningPIDController.getP())
        i = SmartDashboard.getNumber("i",self.frontLeft.turningPIDController.getI())
        d = SmartDashboard.getNumber("d",self.frontLeft.turningPIDController.getD())

        self.frontLeft.updateTurnPID(p,i,d)
        self.frontRight.updateTurnPID(p,i,d)
        self.backLeft.updateTurnPID(p,i,d)
        self.backRight.updateTurnPID(p,i,d)
    def displayDrivePID(self):
        SmartDashboard.getNumber("p",self.frontLeft.drivePIDController.getP())
        SmartDashboard.getNumber("i",self.frontLeft.drivePIDController.getI())
        SmartDashboard.getNumber("d",self.frontLeft.drivePIDController.getD())

    def updateDrivePIDs(self):
        p = SmartDashboard.getNumber("p",self.frontLeft.drivePIDController.getP())
        i = SmartDashboard.getNumber("i",self.frontLeft.drivePIDController.getI())
        d = SmartDashboard.getNumber("d",self.frontLeft.drivePIDController.getD())

        self.frontLeft.updateDrivePID(p,i,d)
        self.frontRight.updateDrivePID(p,i,d)
        self.backLeft.updateDrivePID(p,i,d)
        self.backRight.updateDrivePID(p,i,d)
    
    def displayVoltage(self):
        SmartDashboard.putNumber("drive voltage",self.frontLeft.voltage)

    def updateVoltage(self):
        voltage = SmartDashboard.getNumber("drive voltage", self.frontLeft.voltage)
        self.frontLeft.updateVoltage(voltage)
        self.frontRight.updateVoltage(voltage)
        self.backLeft.updateVoltage(voltage)
        self.backRight.updateVoltage(voltage)
