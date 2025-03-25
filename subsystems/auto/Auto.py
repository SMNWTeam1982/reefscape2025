import wpimath.controller
import wpimath.geometry
import wpimath.trajectory
from pathplannerlib.auto import AutoBuilder, PathPlannerAuto
from pathplannerlib.path import PathPlannerPath, PathConstraints
from pathplannerlib.controller import PPHolonomicDriveController
from pathplannerlib.config import RobotConfig, PIDConstants
import wpimath
from wpilib import DriverStation, SmartDashboard

import math

from ..swerve.Drive import Drivetrain, DriveConstants

from commands2 import Subsystem
class SwerveAuto:
    def __init__(self,driveReference: Drivetrain):
        AutoBuilder.configure(
            driveReference.getPose,
            driveReference.resetPose,
            driveReference.getRelativeSpeeds,
            lambda speeds, feedforwards: driveReference.driveWithChassisSpeeds(speeds),
            PPHolonomicDriveController(
                PIDConstants(1.0,0.0,0.0),
                PIDConstants(1.0,0.0,0.0)
            ),
            RobotConfig.fromGUISettings(),
            lambda: DriverStation.getAlliance() == DriverStation.Alliance.kRed,
            Subsystem() # give it a fake subsystem to satisfy the requirements
        )
        self.autoChooser = AutoBuilder.buildAutoChooser()
        self.driveReference = driveReference
        self.pathCommand = None # no path command initially
        self.done = True
    
        self.autoChooser = AutoBuilder.buildAutoChooser()
        SmartDashboard.putData("Auto Chooser", self.autoChooser)

    def runAuto(self):
        if self.pathCommand.isFinished():
            if self.done == False:
                self.done = True
                self.pathCommand.end(False) # call end manually
            return
        self.pathCommand.execute() # run the command manually

    def generatePathToPose(self, pose: wpimath.geometry.Pose2d):
        constraints = PathConstraints(
            1.0,
            1.0,
            wpimath.units.degreesToRadians(540),
            wpimath.units.degreesToRadians(720)
        )
        self.pathCommand = AutoBuilder.pathfindToPose(
            pose,
            constraints,
            goal_end_vel=0.0,
            rotation_delay_distance=0.0
        )
        self.pathCommand.initialize()

    def getAutoCommand(self):
        return self.autoChooser.getSelected()




    
