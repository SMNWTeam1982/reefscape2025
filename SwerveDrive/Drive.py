#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

import math
import wpilib
import wpimath.geometry
import wpimath.kinematics
import SwerveModule
from phoenix6 import hardware as ctre

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
        self.frontLeft = SwerveModule.SwerveModule(0,0,0)
        self.frontRight = SwerveModule.SwerveModule(0,0,0) # these need CANIDs
        self.backLeft = SwerveModule.SwerveModule(0,0,0)
        self.backRight = SwerveModule.SwerveModule(0,0,0)
        
        self.gyro = ctre.Pigeon2(0) # this needs a CANID

        # unsure if kinematics is constant so I keep here - zach
        self.kinematics = wpimath.kinematics.SwerveDrive4Kinematics(
            self.frontLeftLocation,
            self.frontRightLocation,
            self.backLeftLocation,
            self.backRightLocation,
        )

        self.odometry = wpimath.kinematics.SwerveDrive4Odometry(
            self.kinematics,
            self.gyro.getRotation2d(),
            (
                self.frontLeft.getPosition(),
                self.frontRight.getPosition(),
                self.backLeft.getPosition(),
                self.backRight.getPosition(),
            ),
        )

        self.gyro.reset()

    def drive(
        self,
        xSpeed: float,
        ySpeed: float,
        rot: float,
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
                        xSpeed, ySpeed, rot, self.gyro.getRotation2d()
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
            self.gyro.getRotation2d(),
            (
                self.frontLeft.getPosition(),
                self.frontRight.getPosition(),
                self.backLeft.getPosition(),
                self.backRight.getPosition(),
            ),
        )
