from pathplannerlib.auto import AutoBuilder,PathPlannerAuto
from pathplannerlib.path import PathPlannerPath
from pathplannerlib.controller import PPHolonomicDriveController
from pathplannerlib.config import RobotConfig, PIDConstants
from wpilib import DriverStation
from wpilib import SendableChooser
from wpilib import SmartDashboard
from elevator.Elevator import Elevator
from elevator.Intake import Intake

from swerve.Drive import Drivetrain,DriveConstants

from commands2 import Subsystem
class SwerveAuto:
    def __init__(self,driveReference: Drivetrain):
        AutoBuilder.configureHolonomic(
            driveReference.getPose,
            driveReference.resetPose,
            driveReference.getRelativeSpeeds,
            driveReference.driveWithChassisSpeeds,
            PPHolonomicDriveController(
                PIDConstants(1.0,0.0,0.0),
                PIDConstants(1.0,0.0,0.0),
                DriveConstants.MAX_SPEED_METERS_PER_SECOND,
                DriveConstants.FRONT_LEFT_LOCATION.norm()
            ),
            lambda: DriverStation.getAlliance() == DriverStation.Alliance.kRed,
            Subsystem() # give it a fake subsystem to satisfy the requirements
        )

        self.driveReference = driveReference
        self.pathCommand = AutoBuilder.followPath(PathPlannerPath.fromPathFile("my first auto"))
        self.pathCommand.initialize() # initialize manualy
        self.done = False
        self.chooser = SendableChooser()
        // options, want to test if they actually show up.
        self.chooser.addOption("Drive", Drivetrain())
        self.chooser.addOption("Elevator", Elevator())
        self.chooser.addOption("Intake", Intake())
    
    def runAuto(self):
        if self.pathCommand.isFinished():
            if self.done == False:
                self.done = True
                self.pathCommand.end() # call end manually
            return
        self.pathCommand.execute() # run the command manually
