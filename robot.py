import wpilib
from subsystems.swerve import Drive
from subsystems.auto import Auto


class MyRobot(wpilib.TimedRobot):
    def robotInit(self):
        self.drive = Drive.Drivetrain()
        self.driveController = wpilib.XboxController(0)
        # We are NOT using this at comp - Kay
        #self.guitar = wpilib.XboxController(1)
        self.auto = Auto.SwerveAuto(self.drive)

    def robotPeriodic(self):
        self.drive.updatePoseEstimation()
        self.drive.displayTelemetry()

        if self.driveController.getAButton():
            self.drive.displayDrivePID()
        if self.driveController.getBButton():
            self.drive.updateDrivePIDs()

    def disabledPeriodic(self):
        return super().disabledPeriodic()

    def autonomousInit(self):
        pass
    def autonomousPeriodic(self):
        self.auto.runAuto()
        pass
    def teleopInit(self):
        pass
    def teleopPeriodic(self):
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
