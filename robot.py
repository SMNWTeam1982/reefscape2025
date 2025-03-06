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

class MyRobot(wpilib.TimedRobot):
    def robotInit(self):
        self.powerDistributionModule = wpilib.PowerDistribution(1,wpilib.PowerDistribution.ModuleType.kRev)
        self.drive = Drive.Drivetrain()
        self.elevator = Elevator.Elevator(self.powerDistributionModule)
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
        self.elevator.logElevatorHeight()
        self.elevator.intake.logIntakeCurrents()
        self.elevator.intake.logWristPosition()
        self.drive.logPoseEstimation()

        #if self.driveController.getAButton():
            #self.drive.displayDrivePID()
        #if self.driveController.getBButton():
            #self.drive.updateDrivePIDs()
    def autonomousInit(self):
        pass
    def autonomousPeriodic(self):
        #self.auto.runAuto()
        pass
    def teleopInit(self):
        pass
    def teleopPeriodic(self):

        # for running the PIDs
        if self.driveController.getAButton():
            self.elevator.moveElevator() # runs the elevator pid

        if self.driveController.getXButton(): # run the wrist when pressing X
            self.elevator.intake.runWrist()

        if self.driveController.getBButton(): # zero elevator encoders
            self.elevator.zer0AltitudeEncoders()
        if self.driveController.getYButton(): # zero wrist encoders
            self.elevator.intake.zeroEncoder()
        
        # for running the elevator manually
        if self.driveController.getRightBumper():
            self.elevator.moveElevatorRaw(0.1)
        elif self.driveController.getLeftBumper():
            self.elevator.moveElevatorRaw(-0.1)
        else:
            self.elevator.moveElevatorRaw(0.0)

        # returning early causes the code to not work, I commented everything else out instead
        #return # early return for the sake of testing, this will make it so we can use the controls for other stuff
        

        #if self.operatorController.getButton(2):
        #    self.elevator.setL1()
        #if self.operatorController.getButton(1):
        #    self.elevator.setL2()
        #if self.operatorController.getButton(7):
        #    self.elevator.setL3Coral()
        #if self.operatorController.getButton(6):
        #    self.elevator.setL4()
        #if self.operatorController.getButton(5):
        #    self.elevator.setHighAlgae()
        #if self.operatorController.getButton(6):
        #    self.elevator.setProcessor()
        #if self.operatorController.getButton(7):
        #    self.elevator.setStation()
        #if self.operatorController.getButton(8):
        #    self.elevator.setIdle()
        #if self.operatorController.getButton(9):
        #    self.elevator.setL3Algae()
        #if self.operatorController.getButton(10):
        #    self.elevator.intake.runIntakeEject()
        #if self.driveController.getRightBumper():
        #    self.climber.setRaised()
        #if self.driveController.getLeftBumper():
        #    self.climber.setLowered()

        
    
        # ---------- put operator controls above this line --------------------
        # Commented out for elevator testing - Kay 3/5
        #if self.driveController.getAButton():
        #    self.auto.pathCommand.cancel()
        #    self.runningReefNavigation = False

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
            self.deadzone(turn),
            True
        )
        # Commented out for elevator testing - Kay 3/5
        #if self.driveController.getXButton(): # set left pos
        #    targetPose = ReefNavigator.getNearestLeft(self.drive.getPose())
        #    if targetPose.translation().distance(self.drive.getPose().translation()) < ReefNavigator.ReefNavigationConstants.SNAP_RADIUS:
        #        self.auto.generatePathToPose(targetPose)
        #        self.runningReefNavigation = True
        #if self.driveController.getBButton(): # set right pos
        #    targetPose = ReefNavigator.getNearestRight(self.drive.getPose())
        #    if targetPose.translation().distance(self.drive.getPose().translation()) < ReefNavigator.ReefNavigationConstants.SNAP_RADIUS:
        #        self.auto.generatePathToPose(targetPose)
        #        self.runningReefNavigation = True
        #if self.driveController.getAButton(): # set L1 pos
        #    targetPose = ReefNavigator.getNearestL1Setpoint()
        #    if targetPose.translation().distance(self.drive.getPose().translation()) < ReefNavigator.ReefNavigationConstants.SNAP_RADIUS:
        #        self.auto.generatePathToPose(targetPose)
        #        self.runningReefNavigation = True
    
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
