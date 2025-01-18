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
            self.drive.displayDrivePID()
            self.drive.displayVoltage()
        if self.driveController.getBButton():
            self.drive.updateDrivePIDs()
            self.drive.updateVoltage()
    def autonomousInit(self):
        pass
    def autonomousPeriodic(self):
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
