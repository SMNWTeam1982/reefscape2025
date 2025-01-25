import wpilib
from subsystems.swerve import Drive
from subsystems.auto import Auto


class MyRobot(wpilib.TimedRobot):
    def robotInit(self):
        self.drive = Drive.Drivetrain()
        self.driveController = wpilib.XboxController(0)
        #self.auto = Auto.SwerveAuto(self.drive)

    def robotPeriodic(self):
        
        self.drive.displayTelemetry()

        if self.driveController.getAButton():
            self.drive.displayDrivePID()
        if self.driveController.getBButton():
            self.drive.updateDrivePIDs()

    def autonomousInit(self):
        pass
    def autonomousPeriodic(self):
        #self.auto.runAuto()
        pass
    def teleopInit(self):
        pass
    def teleopPeriodic(self):
        x = -self.driveController.getLeftX()
        y = self.driveController.getLeftY()
        turn = self.driveController.getRightX()

        self.drive.drive(
            self.deadzone(x),
            self.deadzone(y),
            self.deadzone(turn),
            self.getPeriod()
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
