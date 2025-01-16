import math
import wpilib
import wpimath.kinematics
from swerve import Drive

class MyRobot(wpilib.TimedRobot):
    def robotInit(self):
        self.drive = Drive.Drivetrain()
        self.driveController = wpilib.XboxController(0)
    def robotPeriodic(self):
        self.drive.displayTelemetry()

        if self.driveController.getAButton():
            self.drive.displayPID()
        if self.driveController.getBButton():
            self.drive.updatePIDs()
    def autonomousInit(self):
        pass
    def autonomousPeriodic(self):
        pass
    def teleopInit(self):
        pass
    def teleopPeriodic(self):
        x = self.driveController.getLeftX()
        y = self.driveController.getLeftY()
        turn = self.driveController.getRightX()

        self.drive.drive(
            self.deadzone(x),
            self.deadzone(y),
            self.deadzone(turn),
            self.getPeriod()
        )
    
    def deadzone(num: float) -> float:
        if num.__abs__() < 0.05:
            return 0.0

    def testInit(self):
        pass
    def testPeriodic(self):
        pass

if __name__ == "__main__":
    wpilib.run(MyRobot)
