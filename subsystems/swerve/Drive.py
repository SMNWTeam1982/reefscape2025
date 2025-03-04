#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#
import wpimath.units
import math
import wpimath.geometry
from wpilib import Field2d
import wpimath.kinematics
from . import SwerveModule
from phoenix6 import hardware as ctre
from photonlibpy.photonCamera import PhotonCamera
from photonlibpy.photonPoseEstimator import PhotonPoseEstimator, PoseStrategy
import robotpy_apriltag
from pathplannerlib import DriveFeedforwards
from pathplannerlib.logging import PathPlannerLogging

from wpimath.estimator import SwerveDrive4PoseEstimator


from wpilib import SmartDashboard

class DriveConstants:
    # this is the physical max speed not a speed limit
    PHYSICAL_MAX_SPEED_METERS_PER_SECOND = 3.8 # speed at 12 Volts, Jan 18 2025

    SPEED_CAP_METERS_PER_SECOND = 2.5 # arbitrary cap

    # translation values taken from 2024 code
    FRONT_LEFT_LOCATION = wpimath.geometry.Translation2d(0.2635, 0.2635)
    FRONT_RIGHT_LOCATION = wpimath.geometry.Translation2d(0.2635, -0.2635)
    BACK_LEFT_LOCATION = wpimath.geometry.Translation2d(-0.2635, 0.2635)
    BACK_RIGHT_LOCATION = wpimath.geometry.Translation2d(-0.2635, -0.2635)

    CAMERA_POSITION_RELATIVE_TO_ROBOT = wpimath.geometry.Transform3d(
        wpimath.units.inchesToMeters(15.0),
        0.0,
        wpimath.units.inchesToMeters(5.0),
        wpimath.geometry.Rotation3d.fromDegrees(0.0,11.0,3.5)
    )

class Drivetrain:
    """
    Represents a swerve drive style drivetrain.
    """

    def __init__(self) -> None:
        
        self.frontLeft = SwerveModule.Wheel(7,8,4)
        self.frontRight = SwerveModule.Wheel(1,2,3)
        self.backLeft = SwerveModule.Wheel(5,4,1)
        self.backRight = SwerveModule.Wheel(3,6,2)
        
        self.gyro = ctre.pigeon2.Pigeon2(0)

        self.kinematics = wpimath.kinematics.SwerveDrive4Kinematics(
            DriveConstants.FRONT_LEFT_LOCATION,
            DriveConstants.FRONT_RIGHT_LOCATION,
            DriveConstants.BACK_LEFT_LOCATION,
            DriveConstants.BACK_RIGHT_LOCATION,
        )

        self.gyro.set_yaw(0)

        self.cam = PhotonCamera('limelight-front')

        self.photonVisionPoseEstimator = PhotonPoseEstimator(
            robotpy_apriltag.AprilTagFieldLayout.loadField(robotpy_apriltag.AprilTagField.kDefaultField),
            PoseStrategy.LOWEST_AMBIGUITY,
            self.cam,
            DriveConstants.CAMERA_POSITION_RELATIVE_TO_ROBOT
        )

        self.poseEstimator = SwerveDrive4PoseEstimator(
            self.kinematics,
            wpimath.geometry.Rotation2d.fromDegrees(self.gyro.get_yaw().value),
            (
                self.frontLeft.getPosition(),
                self.frontRight.getPosition(),
                self.backLeft.getPosition(),
                self.backRight.getPosition(),
            ),
            wpimath.geometry.Pose2d(),
            # closer to 0 is more trust
            (0.1, 0.1, 0.1), # trust swerve module data slightly less, except for gyro
            (0.09, 0.09, 1) # trust vision data slightly more
        )

        self.field = Field2d()
        SmartDashboard.putData("Field", self.field)

    def driveWithChassisSpeeds(self,speeds: wpimath.kinematics.ChassisSpeeds):
        self.drive( # currently we are supplying robot relative speeds to a field relative function (not good)
            speeds.vx,
            speeds.vy,
            speeds.omega,
            False
        )

        self.logPathplannerChassisSpeeds(speeds)

    def logPathplannerChassisSpeeds(speeds: wpimath.kinematics.ChassisSpeeds):
        SmartDashboard.putNumber("pathplanner vx", speeds.vx)
        SmartDashboard.putNumber("pathplanner vy", speeds.vy)
        SmartDashboard.putNumber("pathplanner omega", speeds.omega)

    def drive(
        self,
        xSpeed: float, # meters per second
        ySpeed: float, # meters per second
        rotation: float, # radians per second
        fieldRelative: bool
    ) -> None:
        """
        Method to drive the robot using joystick info.
        :param xSpeed: Speed of the robot in the x direction (forward).
        :param ySpeed: Speed of the robot in the y direction (sideways).
        :param rot: Angular rate of the robot.
        """

        speedsToDiscretize = None
        if fieldRelative:
            speedsToDiscretize = wpimath.kinematics.ChassisSpeeds.fromFieldRelativeSpeeds(
                xSpeed, ySpeed, rotation, wpimath.geometry.Rotation2d.fromDegrees(self.gyro.get_yaw().value)
            )
        else:
            speedsToDiscretize = wpimath.kinematics.ChassisSpeeds(
                xSpeed,ySpeed,rotation
            )
        swerveModuleStates = self.kinematics.toSwerveModuleStates(
            wpimath.kinematics.ChassisSpeeds.discretize(
                (
                    speedsToDiscretize
                ),
                0.02, # default period
            )
        )
        
        wpimath.kinematics.SwerveDrive4Kinematics.desaturateWheelSpeeds(
            swerveModuleStates, DriveConstants.SPEED_CAP_METERS_PER_SECOND
        ) # cap speed
        
        self.frontLeft.setDesiredState(swerveModuleStates[0])
        self.frontRight.setDesiredState(swerveModuleStates[1])
        self.backLeft.setDesiredState(swerveModuleStates[2])
        self.backRight.setDesiredState(swerveModuleStates[3])

        SmartDashboard.putNumberArray(
            "Desired Module States",
            [
                swerveModuleStates[0].angle.radians(),swerveModuleStates[0].speed,
                swerveModuleStates[1].angle.radians(),swerveModuleStates[1].speed,
                swerveModuleStates[2].angle.radians(),swerveModuleStates[2].speed,
                swerveModuleStates[3].angle.radians(),swerveModuleStates[3].speed
            ]
        )

        currentStates = [
            self.frontLeft.getState(),
            self.frontRight.getState(),
            self.backLeft.getState(),
            self.backRight.getState()
        ]

        SmartDashboard.putNumberArray(
            "Current Module States",
            [
                currentStates[0].angle.radians(),currentStates[0].speed,
                currentStates[1].angle.radians(),currentStates[1].speed,
                currentStates[2].angle.radians(),currentStates[2].speed,
                currentStates[3].angle.radians(),currentStates[3].speed
            ]
        )

    def updatePoseEstimation(self) -> None:
        self.poseEstimator.update(
            wpimath.geometry.Rotation2d.fromDegrees(self.gyro.get_yaw().value),
            (
                self.frontLeft.getPosition(),
                self.frontRight.getPosition(),
                self.backLeft.getPosition(),
                self.backRight.getPosition(),
            )
        )

        camResult = self.cam.getLatestResult()

        result = self.photonVisionPoseEstimator.update(camResult)
        if result:
            self.poseEstimator.addVisionMeasurement(result.estimatedPose.toPose2d(),result.timestampSeconds)

        self.field.setRobotPose(self.poseEstimator.getEstimatedPosition())
        SmartDashboard.putBoolean("target aquired",camResult.hasTargets())

    def getPose(self) -> wpimath.geometry.Pose2d:
        return self.poseEstimator.getEstimatedPosition() # we will get the robot pose from vision
    
    def logPoseEstimation(self):
        self.field.setRobotPose(self.getPose())
    
    def resetPose(self,pose: wpimath.geometry.Pose2d):
        self.poseEstimator.resetPosition(
            wpimath.geometry.Rotation2d.fromDegrees(self.gyro.get_yaw().value),
            (
                self.frontLeft.getPosition(),
                self.frontRight.getPosition(),
                self.backLeft.getPosition(),
                self.backRight.getPosition(),
            ),
            pose
        )
    def getRelativeSpeeds(self) -> wpimath.kinematics.ChassisSpeeds:
        moduleStates = [
            self.frontLeft.getState(),
            self.frontRight.getState(),
            self.backLeft.getState(),
            self.backRight.getState()
        ]
        return self.kinematics.toChassisSpeeds(moduleStates)

    def displayTelemetry(self):
        pass

    def displayTurnPID(self):
        SmartDashboard.putNumber("turn p",self.frontLeft.turningPIDController.getP())
        SmartDashboard.putNumber("turn i",self.frontLeft.turningPIDController.getI())
        SmartDashboard.putNumber("turn d",self.frontLeft.turningPIDController.getD())
    
    def updateTurnPIDs(self):
        p = SmartDashboard.getNumber("turn p",self.frontLeft.turningPIDController.getP())
        i = SmartDashboard.getNumber("turn i",self.frontLeft.turningPIDController.getI())
        d = SmartDashboard.getNumber("turn d",self.frontLeft.turningPIDController.getD())

        self.frontLeft.updateTurnPID(p,i,d)
        self.frontRight.updateTurnPID(p,i,d)
        self.backLeft.updateTurnPID(p,i,d)
        self.backRight.updateTurnPID(p,i,d)
    def displayDrivePID(self):
        SmartDashboard.getNumber("drive p",self.frontLeft.drivePIDController.getP())
        SmartDashboard.getNumber("drive i",self.frontLeft.drivePIDController.getI())
        SmartDashboard.getNumber("drive d",self.frontLeft.drivePIDController.getD())

    def updateDrivePIDs(self):
        p = SmartDashboard.getNumber("drive p",self.frontLeft.drivePIDController.getP())
        i = SmartDashboard.getNumber("drive i",self.frontLeft.drivePIDController.getI())
        d = SmartDashboard.getNumber("drive d",self.frontLeft.drivePIDController.getD())

        self.frontLeft.updateDrivePID(p,i,d)
        self.frontRight.updateDrivePID(p,i,d)
        self.backLeft.updateDrivePID(p,i,d)
        self.backRight.updateDrivePID(p,i,d)