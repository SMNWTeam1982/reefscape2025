from pathplannerlib.auto import AutoBuilder, PathPlannerAuto
from pathplannerlib.path import PathPlannerPath
from pathplannerlib.controller import PPHolonomicDriveController
from pathplannerlib.config import RobotConfig, PIDConstants
from wpilib import DriverStation

from ..swerve.Drive import Drivetrain, DriveConstants

from commands2 import Subsystem
class SwerveAuto:
    def __init__(self,driveReference: Drivetrain):
        AutoBuilder.configure(
            driveReference.getPose,
            driveReference.resetPose,
            driveReference.getRelativeSpeeds,
            driveReference.driveWithChassisSpeeds,
            PPHolonomicDriveController(
                PIDConstants(1.0,0.0,0.0),
                PIDConstants(1.0,0.0,0.0)
            ),
            RobotConfig.fromGUISettings(),
            lambda: DriverStation.getAlliance() == DriverStation.Alliance.kRed,
            Subsystem() # give it a fake subsystem to satisfy the requirements
        )

        self.driveReference = driveReference
        self.pathCommand = AutoBuilder.followPath(PathPlannerPath.fromPathFile("Example Path"))
        self.pathCommand.initialize() # initialize manualy
        self.done = False
    
    def runAuto(self):
        if self.pathCommand.isFinished():
            if self.done == False:
                self.done = True
                self.pathCommand.end(False) # call end manually
            return
        self.pathCommand.execute() # run the command manually
