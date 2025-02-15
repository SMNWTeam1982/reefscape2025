import math
import wpilib
import wpimath.kinematics
import wpimath
import auto.ReefNavigator
import auto.SwerveAuto
from swerve import Drive
import auto
# from photonlibpy.photonPoseEstimator import PoseStrategy
# from robotpy_apriltag import AprilTagField, AprilTagFieldLayout

# this value needs to be adjusted for the actual robot
kRobotToCam = wpimath.geometry.Transform3d(
    wpimath.geometry.Translation3d(.5, 0, .5),
    wpimath.geometry.Rotation3d.fromDegrees(0, -30, 0)
)


class MyRobot(wpilib.TimedRobot):
    def robotInit(self):
        self.drive = Drive.Drivetrain()
        self.driveController = wpilib.XboxController(0)
        self.operateController = wpilib.XboxController(1)
        self.auto = auto.SwerveAuto.SwerveAuto(self.drive)
        self.runningReefNavigation = False

        
#        self.cam = PhotonCamera("Camera_Module_v1")
#        self.camPoseEst = PhotonPoseEstimator(
#            AprilTagFieldLayout.loadField(AprilTagField.kDefaultField),
#            PoseStrategy.LOWEST_AMBIGUITY,
#            self.cam,
#            kRobotToCam,
#        )
    def robotPeriodic(self):
#        camEstPose = self.camPoseEst.update()

        # update pose with this (probably doesnt work)
        # if camEstPose:
        #     self.drive.addVisionPoseEstimate(
        #             camEstPose.estimatedPos, camEstPose.timestampSeconds
        #     )




        self.drive.displayTelemetry()

        if self.driveController.getAButton():
            self.drive.displayDrivePID()
        if self.driveController.getBButton():
            self.drive.updateDrivePIDs()

        
        
    def autonomousInit(self):
        pass
    def autonomousPeriodic(self):
        self.auto.runAuto()
    def teleopInit(self):
        pass
    def teleopPeriodic(self):

        # put operator controls above this if stanement

        if self.runningReefNavigation:
            if self.auto.runAuto():
                self.runningReefNavigation = False
            else:
                return



        x = -self.driveController.getLeftX()
        y = self.driveController.getLeftY()
        turn = self.driveController.getRightX()

        self.drive.drive(
            self.deadzone(x),
            self.deadzone(y),
            self.deadzone(turn),
            True
        )
        
        if self.operateController.getAButton():
            targetPose = auto.ReefNavigator.getNearestLeft()
            if targetPose.translation().distance(self.drive.getPose().translation()) < auto.ReefNavigator.ReefNavigationConstants.SNAP_RADIUS:
                self.auto.generatePathToPose(targetPose)
                self.runningReefNavigation = True
        if self.operateController.getBButton():
            targetPose = auto.ReefNavigator.getNearestRight()
            if targetPose.translation().distance(self.drive.getPose().translation()) < auto.ReefNavigator.ReefNavigationConstants.SNAP_RADIUS:
                self.auto.generatePathToPose(targetPose)
                self.runningReefNavigation = True
    
    def deadzone(self, num: float) -> float:
        if abs(num) < 0.05:
            return 0.0
        return num

    def testInit(self):
        pass
    def testPeriodic(self):
        pass

if __name__ == "__main__":
    wpilib.run(MyRobot)
