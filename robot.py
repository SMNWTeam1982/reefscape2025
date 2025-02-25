import math
import wpilib
import wpimath.kinematics
import wpimath
from subsystems.swerve import Drive
from subsystems.auto import Auto, ReefNavigator
from subsystems.elevator import Elevator
from subsystems.climber import Climber
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
        self.elevator = Elevator.Elevator()
        self.climber = Climber.Climber()
        self.driveController = wpilib.XboxController(0)
        # We are NOT using this at comp - Kay
        #self.guitar = wpilib.XboxController(1)
        self.operatorController = wpilib.XboxController(1)

        self.elevatorTimer = wpilib.Timer()
        self.climberTimer = wpilib.Timer()

        self.auto = Auto.SwerveAuto(self.drive)
        self.runningReefNavigation = False

    def robotPeriodic(self):
        self.drive.displayTelemetry()

        #if self.driveController.getAButton():
            #self.drive.displayDrivePID()
        #if self.driveController.getBButton():
            #self.drive.updateDrivePIDs()
    def autonomousInit(self):
        pass
    def autonomousPeriodic(self):
        self.auto.runAuto()
        pass
    def teleopInit(self):
        pass
    def teleopPeriodic(self):
        # makes it so that you cant quickly change in between elevator states

        if self.elevatorTimer.isRunning():
            if self.elevatorTimer.get() > 1: # this number is the cooldown between state change
                self.elevatorTimer.stop()
                self.elevatorTimer.reset()
        else:
            if self.operatorController.getButton(1):
                self.elevator.setL1()
                self.elevatorTimer.start()
            if self.operatorController.getButton(2):
                self.elevator.setL2()
                self.elevatorTimer.start()
            if self.operatorController.getButton(3):
                self.elevator.setL3()
                self.elevatorTimer.start()
            if self.operatorController.getButton(4):
                self.elevator.setL4()
                self.elevatorTimer.start()
            if self.operatorController.getButton(5):
                self.elevator.setHighAlgae()
                self.elevatorTimer.start()
            if self.operatorController.getButton(6):
                self.elevator.setProcessor()
                self.elevatorTimer.start()
            if self.operatorController.getButton(7):
                self.elevator.setStation()
                self.elevatorTimer.start()
            if self.operatorController.getButton(8):
                self.elevator.setIdle()
                self.elevatorTimer.start()

        if self.driveController.getRightBumper():
            self.climber.setRaised()
        if self.driveController.getLeftBumper():
            self.climber.setLowered()
    
        # ---------- put operator controls above this line --------------------

        if self.runningReefNavigation:
            if self.auto.runAuto(): # check if its done and end nav when it is
                self.runningReefNavigation = False
            else:
                return # dont let the driver have control while nav is runnig

        # ---------- driver controls below this line --------------------------

        x = -self.driveController.getLeftX()
        y = self.driveController.getLeftY()
        turn = self.driveController.getRightX()

        self.drive.drive(
            self.deadzone(x),
            self.deadzone(y),
            self.deadzone(turn)
        )

        if self.driveController.getXButton():
            targetPose = ReefNavigator.getNearestLeft(self.drive.getPose())
            if targetPose.translation().distance(self.drive.getPose().translation()) < ReefNavigator.ReefNavigationConstants.SNAP_RADIUS:
                self.auto.generatePathToPose(targetPose)
                self.runningReefNavigation = True
        if self.driveController.getBButton():
            targetPose = ReefNavigator.getNearestRight(self.drive.getPose())
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
