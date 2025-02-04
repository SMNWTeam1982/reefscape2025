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
import photonlibpy
import robotpy_apriltag
from pathplannerlib.util import DriveFeedforwards
from pathplannerlib.logging import PathPlannerLogging

from wpimath.estimator import SwerveDrive4PoseEstimator


from wpilib import SmartDashboard

class DriveConstants:
    # this is the physical max speed not a speed limit
    MAX_SPEED_METERS_PER_SECOND = 3.8 # speed at 12 Volts, Jan 18 2025
    
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

        self.cam = PhotonCamera('Camera_Module_v1')

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
            (0.1,0.1,0.1), # trust swerve module data slightly less, except for gyro
            (0.09,0.09,1) # trust vision data slightly more
        )

        self.field = Field2d()
        SmartDashboard.putData("Field", self.field)

    def driveWithChassisSpeeds(self,speeds: wpimath.kinematics.ChassisSpeeds,feeds: DriveFeedforwards):
        self.drive(
            speeds.vx,
            speeds.vy,
            speeds.omega
        )

    def drive(
        self,
        xSpeed: float, # meters per second
        ySpeed: float, # meters per second
        rotation: float, # radians per second
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
                0.02,
            )
        )
        
        wpimath.kinematics.SwerveDrive4Kinematics.desaturateWheelSpeeds(
            swerveModuleStates, DriveConstants.MAX_SPEED_METERS_PER_SECOND
        )
        
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

        result = self.photonVisionPoseEstimator.update(self.cam.getLatestResult())
        if result:
            self.poseEstimator.addVisionMeasurement(result.estimatedPose.toPose2d(),result.timestampSeconds)

    def getPose(self) -> wpimath.geometry.Pose2d:
        return self.poseEstimator.getEstimatedPosition() # we will get the robot pose from vision
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
    
    def displayTelemetry(self) -> None:
        SmartDashboard.putNumber("gyro angle",math.radians(self.gyro.get_yaw().value))

        SmartDashboard.putNumber("front left angle error",self.frontLeft.turningPIDController.getPositionError())# self.frontLeft.getPosition().angle.degrees())
        #SmartDashboard.putNumber("front left velocity error",self.frontLeft.drivePIDController.getPositionError())
        
        SmartDashboard.putNumber("front right angle error",self.frontRight.turningPIDController.getPositionError())# self.frontRight.getPosition().angle.degrees())
        #SmartDashboard.putNumber("front right velocity error",self.frontRight.drivePIDController.getPositionError())
        
        SmartDashboard.putNumber("back left angle error",self.backLeft.turningPIDController.getPositionError())# self.backLeft.getPosition().angle.degrees())
        #SmartDashboard.putNumber("back left velocity error",self.backLeft.drivePIDController.getPositionError())
        
        SmartDashboard.putNumber("back right angle error",self.backRight.turningPIDController.getPositionError())# self.backRight.getPosition().angle.degrees())
        #SmartDashboard.putNumber("back right velocity error",self.backRight.drivePIDController.getPositionError())
        
        moduleStates = [
            self.frontLeft.getState(),
            self.frontRight.getState(),
            self.backLeft.getState(),
            self.backRight.getState()
        ]

        SmartDashboard.putNumberArray(
            "Module States",
            [
                moduleStates[0].angle.radians(),moduleStates[0].speed,
                moduleStates[1].angle.radians(),moduleStates[1].speed,
                moduleStates[2].angle.radians(),moduleStates[2].speed,
                moduleStates[3].angle.radians(),moduleStates[3].speed
            ]
        )

        #PathPlannerLogging.setLogCurrentPoseCallback(lambda pose: self.field.setRobotPose(pose))
        #PathPlannerLogging.setLogTargetPoseCallback(lambda pose: self.field.getObject("target pose").setPose(pose))
        #PathPlannerLogging.setLogActivePathCallback(lambda poses: self.field.getObject("path").setPoses(poses))

        #SmartDashboard.putData("robot pose",self.poseEstimator.getEstimatedPosition())
        pose = self.poseEstimator.getEstimatedPosition()
        SmartDashboard.putNumber("x",pose.x)
        SmartDashboard.putNumber("y",pose.y)
        self.field.setRobotPose(pose)
        
        SmartDashboard.putBoolean("targets",self.cam.getLatestResult().hasTargets())

        # SmartDashboard.putData(
        #     "module positions",
        #     [
        #         self.frontLeft.getPosition(),
        #         self.frontRight.getPosition(),
        #         self.backLeft.getPosition(),
        #         self.backRight.getPosition()
        #     ]
        # )

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