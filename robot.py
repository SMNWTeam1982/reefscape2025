import math
import wpilib
import wpimath.kinematics
import wpimath
from subsystems.swerve import Drive
from subsystems.auto import Auto, ReefNavigator
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
        # We are NOT using this at comp - Kay
        #self.guitar = wpilib.XboxController(1)
        self.operateController = wpilib.XboxController(1)
        self.auto = Auto.SwerveAuto(self.drive)
        self.runningReefNavigation = False

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
        pass
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
        # Unused Guitar code - see above
        #x=0.0
        #y=0.0
        #turn=0.0
        #if self.guitar.getPOV() == 0:
            #turn = 1
        #if self.guitar.getPOV() == 180:
            #turn = -1
        #if self.guitar.getAButton():
            #x+=1
        #if self.guitar.getBButton():
            #y-=1
        #if self.guitar.getXButton():
            #x-=1
        #if self.guitar.getYButton():
            #y+=1

        self.drive.drive(
            self.deadzone(x),
            self.deadzone(y),
            self.deadzone(turn)
        )
        if self.operateController.getAButton():
            targetPose = ReefNavigator.getNearestLeft()
            if targetPose.translation().distance(self.drive.getPose().translation()) < ReefNavigator.ReefNavigationConstants.SNAP_RADIUS:
                self.auto.generatePathToPose(targetPose)
                self.runningReefNavigation = True
        if self.operateController.getBButton():
            targetPose = ReefNavigator.getNearestRight()
            if targetPose.translation().distance(self.drive.getPose().translation()) < ReefNavigator.ReefNavigationConstants.SNAP_RADIUS:
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
