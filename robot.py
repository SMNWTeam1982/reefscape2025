import math
import wpilib
import wpimath.controller
import wpimath.filter
import wpimath.kinematics
import wpimath
import wpimath.trajectory
from subsystems.swerve import Drive
from subsystems.auto import Auto, ReefNavigator
from subsystems.elevator import Elevator
from subsystems.elevator import Intake
from subsystems.climber import Climber
from wpilib import SmartDashboard
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
        self.nav = ReefNavigator.Navigator(self.drive)

        self.yRateLimiter = wpimath.filter.SlewRateLimiter(5.0,-5.0,0)
        self.xRateLimiter = wpimath.filter.SlewRateLimiter(5.0,-5.0,0)
        self.thetaRateLimiter = wpimath.filter.SlewRateLimiter(5.0,-5.0,0)

        
        self.runningReefNavigation = False

    def robotPeriodic(self):
        self.drive.updatePoseEstimation()
        
        
        self.elevator.logElevatorHeight()
        self.elevator.LogRawElevatorHeights()
        self.elevator.logElevatorCurrents()
        
        self.elevator.intake.logIntakeCurrents()
        
        self.elevator.logStateMachineState()
        
        self.climber.logClimberCurrents()
        
        self.elevator.intake.logWristPosition()
        self.elevator.intake.logWristSafety()
        
        self.drive.logPoseEstimation()
        
        #self.livePIDTuning()


    def livePIDTuning(self):
        # set the controller below to the one desired for live pid tuning

        pidController = self.nav.distancePID
        feedforwardController = self.elevator.intake.coralWristFeedForeward

        if self.driveController.getPOV() == 0:
            p = pidController.getP()
            i = pidController.getI()
            d = pidController.getD()
            
            g = feedforwardController.getKg()

            SmartDashboard.putNumber("p",p)
            SmartDashboard.putNumber("i",i)
            SmartDashboard.putNumber("d",d)
            
            SmartDashboard.putNumber("g",g)

        if self.driveController.getPOV() == 180:
            p = SmartDashboard.getNumber("p",0)
            i = SmartDashboard.getNumber("i",0)
            d = SmartDashboard.getNumber("d",0)
            
            g = SmartDashboard.getNumber("g",0)

            pidController.setPID(p,i,d)
            
            feedforwardController.setKg(g)
        
        SmartDashboard.putNumber("p error", pidController.getPositionError())
        SmartDashboard.putNumber("i error", pidController.getAccumulatedError())
        SmartDashboard.putNumber("d error", pidController.getVelocityError())

    def autonomousInit(self):
        self.drive.fieldOrient() # UNTESTED FUNCTION !!! UNTESTED FUNCTION !!! UNTESTED FUNCTION !!!
        self.elevator.zer0AltitudeEncoders()
        self.elevator.intake.zeroEncoder()
        self.autoTimer = wpilib.Timer()
        self.autoTimer.start()
            
    def autonomousPeriodic(self): # UNTESTED AUTO !!! UNTESTED AUTO !!! UNTESTED AUTO !!!
        self.elevator.runStateMachine()
        if self.autoTimer.get() < 5:
            self.drive.drive(0.21,0.0,0.0,False) 

    def teleopInit(self):
        pass
    def teleopPeriodic(self):
        self.elevator.runStateMachine()
    
        if self.operatorController.getRawButton(10): ####
            self.elevator.setL1()
        if self.operatorController.getRawButton(6): ####
            self.elevator.setL2()
        if self.operatorController.getRawButton(5): ####
            self.elevator.setL3Coral()
        if self.operatorController.getRawButton(4): ####
            self.elevator.setL3Algae()
        if self.operatorController.getRawButton(3): ####
            self.elevator.setHighAlgae()
        if self.operatorController.getRawButton(8): ####
            self.elevator.setL4()
        if self.operatorController.getRawButton(7): ####
            self.elevator.setStation()
        # if self.operatorController.getRawButton(8):
        #     self.elevator.setIdle()

        self.elevator.intake.runIntakeEject(self.operatorController.getRawButton(2)) ####

        # if self.operatorController.getRawButton(9):
        #     self.elevator.intake.coralMotor.set(-0.3) # intake
        #     self.elevator.intake.leftAlgaeMotor.set(0.0)
        # elif self.operatorController.getRawButton(10):
        #     self.elevator.intake.coralMotor.set(0.6) # eject
        #     self.elevator.intake.leftAlgaeMotor.set(-0.5)
        # else: 
        #     self.elevator.intake.coralMotor.set(0.0)
        #     self.elevator.intake.leftAlgaeMotor.set(0.0)

        if self.operatorController.getRawButton(11): # change * 50 robot cycles per second ####
            self.elevator.targetHeight += 0.002 # 0.10m/s
        elif self.operatorController.getRawButton(12): ####
            self.elevator.targetHeight -= 0.002 # -0.10m/s

        # we need to reduce on button count
        # 1. fix the intake state machine we can make intake/eject into 1 button (-1 overall buttons) # done
        # 2. have the robot automatically set to idle on intake/eject button release (-1 overall buttons) # done, dont need to set to idle state in first place
        # 3. get very precise setpoints so we can remove manual adjustment (-2 overall buttons)

        
    
        # ---------- put operator controls above this line --------------------
        # Commented out for elevator testing - Kay 3/5
        #if self.driveController.getAButton():
        #    self.auto.pathCommand.cancel()
        #    self.runningReefNavigation = False

        # if self.runningReefNavigation:
        #     if self.auto.runAuto(): # check if its done and end nav when it is
        #         self.runningReefNavigation = False
        #     else:
        #         return # dont let the driver have control while nav is runnig

        # ---------- driver controls below this line --------------------------
        
        if self.driveController.getBButton():
            self.nav.travel(ReefNavigator.getNearestLeft(self.drive.getPose()))
            return
        elif self.driveController.getXButton():
            self.nav.travel(ReefNavigator.getNearestRight(self.drive.getPose()))
            return
        
        

        pov = self.driveController.getPOV()

        if pov == 0:
            self.drive.drive(0.5, 0.0, 0.0, False)
        elif pov == 90:
            self.drive.drive(0.0, -0.3, 0.0, False)
        elif pov == 270:
            self.drive.drive(0.0, 0.3, 0.0, False)
        elif pov == 180:
            self.drive.drive(-0.5, 0.0, 0.0, False)
        else:

            x = -self.driveController.getLeftY()
            y = -self.driveController.getLeftX()
            turn = self.driveController.getRightX()

            if wpilib.DriverStation.getAlliance() == wpilib.DriverStation.Alliance.kRed:
                x = -x
                y = -y

            self.drive.drive(
                self.deadzone(x), 
                self.deadzone(y),
                self.deadzone(turn) * 3,
                True
            )

        if self.driveController.getRightBumper():
            self.climber.runClimberRaw(0.5)
        elif self.driveController.getLeftBumper():
            self.climber.runClimberRaw(-0.5)
        else:
            self.climber.runClimberRaw(0.0)
                
        
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
        
        if self.driveController.getRightBumper():
            self.elevator.moveElevatorRaw(0.2)
        elif self.driveController.getLeftBumper():
            self.elevator.moveElevatorRaw(-0.2)
        else:
            self.elevator.moveElevatorRaw(0.0)

if __name__ == "__main__":
    wpilib.run(MyRobot)
