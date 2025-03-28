import wpimath.units
import math
import wpilib
import wpimath.geometry
import wpimath.kinematics
import robotpy_apriltag

import wpimath.controller
import wpimath.trajectory
import wpimath

from ..swerve.Drive import Drivetrain, DriveConstants



def generateBranchReefSetpoint(idOfTagOnFace: int, rightBranch: bool) -> wpimath.geometry.Pose2d:

    tagPose = robotpy_apriltag.AprilTagFieldLayout.getTagPose(robotpy_apriltag.AprilTagFieldLayout.loadField(robotpy_apriltag.AprilTagField.kDefaultField), idOfTagOnFace).toPose2d() # get the pose of the tag we want

    verticalShift = -3.25 # offset of the intake

    if rightBranch: # shift to the left or right branch
        verticalShift -= 6.5
    else:
        verticalShift += 6.5
    
    westRightTranslation = wpimath.geometry.Translation2d( # based off of tag 18 on the west facing side of the blue reef
        wpimath.units.inchesToMeters(20), # shift away 18 in
        wpimath.units.inchesToMeters(verticalShift),
    )

    newTranslation = wpimath.geometry.Translation2d(
        westRightTranslation.norm(),
        westRightTranslation.angle() + tagPose.rotation()
    )

    return wpimath.geometry.Pose2d(
        tagPose.X() + newTranslation.X(),
        tagPose.Y() + newTranslation.Y(),
        tagPose.rotation() + wpimath.geometry.Rotation2d.fromDegrees(180.0)
    )
        
class ReefNavigationConstants:
    SNAP_RADIUS = wpimath.units.inchesToMeters(25) # set to a slightly larger radius because of the new setpoints, old: 23.87490776


    BLUE_RIGHT_STATION_CENTER = wpimath.geometry.Pose2d(
        wpimath.units.inchesToMeters(41.73899),
        wpimath.units.inchesToMeters(37.126),
        wpimath.geometry.Rotation2d.fromDegrees(54)
    )

    BLUE_RIGHT_GROOVE_OFFSET = wpimath.geometry.Transform2d(
        wpimath.geometry.Translation2d(
            wpimath.units.inchesToMeters(8),
            wpimath.geometry.Rotation2d.fromDegrees(54-90)
        ),
        wpimath.geometry.Rotation2d.fromDegrees(0)
    )
    # comment out if it doesn't work - zach march 5
    REEF_RIGHT_SETPOINTS = [
        generateBranchReefSetpoint(6,True),
        generateBranchReefSetpoint(7,True),
        generateBranchReefSetpoint(8,True), # red right setpoints
        generateBranchReefSetpoint(9,True),
        generateBranchReefSetpoint(10,True),
        generateBranchReefSetpoint(11,True),

        generateBranchReefSetpoint(17,True),
        generateBranchReefSetpoint(18,True),
        generateBranchReefSetpoint(19,True), # blue right setpoints
        generateBranchReefSetpoint(20,True),
        generateBranchReefSetpoint(21,True),
        generateBranchReefSetpoint(22,True),
    ]

    # comment out if no work - zach march 5
    REEF_LEFT_SETPOINTS = [
        generateBranchReefSetpoint(6,False),
        generateBranchReefSetpoint(7,False),
        generateBranchReefSetpoint(8,False), # red left setpoints
        generateBranchReefSetpoint(9,False),
        generateBranchReefSetpoint(10,False),
        generateBranchReefSetpoint(11,False),

        generateBranchReefSetpoint(17,False),
        generateBranchReefSetpoint(18,False),
        generateBranchReefSetpoint(19,False), # blue left setpoints
        generateBranchReefSetpoint(20,False),
        generateBranchReefSetpoint(21,False),
        generateBranchReefSetpoint(22,False)
    ]

    BLUE_L1_SETPOINTS = [
        # :> 
    ]

# dont have L1 points defined
def getNearestL1Setpoint(robotPos: wpimath.geometry.Pose2d) -> wpimath.geometry.Pose2d:
    return wpimath.geometry.Pose2d()
    robotPos.nearest(ReefNavigationConstants.BLUE_L1_SETPOINTS)
def getNearestLeft(robotPos: wpimath.geometry.Pose2d) -> wpimath.geometry.Pose2d:
    return robotPos.nearest(ReefNavigationConstants.REEF_LEFT_SETPOINTS)
def getNearestRight(robotPos: wpimath.geometry.Pose2d) -> wpimath.geometry.Pose2d:
    return robotPos.nearest(ReefNavigationConstants.REEF_RIGHT_SETPOINTS)


def deadzone(num: float) -> float:
    if abs(num) < 0.01:
        return 0.0
    return num


# 
class Navigator:
    def __init__(self,driveReference: Drivetrain):
        self.distancePID = wpimath.controller.ProfiledPIDController(
            2.5,
            0.0,
            0.0,
            wpimath.trajectory.TrapezoidProfile.Constraints(
                0.5, # meters
                1.0
            )
        )

        self.distancePID.reset(0.0,0.0)

        self.rotationPID = wpimath.controller.ProfiledPIDController(
            6,
            0.0,
            0.0,
            wpimath.trajectory.TrapezoidProfile.Constraints(
                2.0, # radians
                1.0
            )
        )

        self.rotationPID.enableContinuousInput(-math.pi,math.pi)

        self.rotationPID.reset(driveReference.getPose().rotation().radians(),0)

        self.driveReference = driveReference
    
    def reset(self): # for resetting
        self.distancePID.reset(0.0,0.0)
        self.rotationPID.reset(self.driveReference.getPose().rotation().radians(),0)


    def travel(self, targetPose: wpimath.geometry.Pose2d): # travels in a straight line to the target
        currentRobotPose = self.driveReference.getPose()
        
        wpilib.SmartDashboard.putNumberArray("target Pose", [targetPose.X(),targetPose.Y(),targetPose.rotation().radians()])

        distanceError = currentRobotPose.translation().distance(targetPose.translation())
        travelDirectionFieldRelative = (targetPose.translation() - currentRobotPose.translation()).angle()

        travelVector = wpimath.geometry.Translation2d(
            self.distancePID.calculate(distanceError,0.0),
            travelDirectionFieldRelative
        )

        self.driveReference.drive(
            -travelVector.X(),
            -travelVector.Y(),
            self.rotationPID.calculate(
                currentRobotPose.rotation().radians(),
                targetPose.rotation().radians()
            ),
            True
        )
        


        