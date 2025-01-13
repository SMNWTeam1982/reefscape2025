import wpilib
import wpimath.kinematics
from swerve import Drive

class MyRobot(wpilib.TimedRobot):
    def robotInit(self):
        self.drive = Drive.Drivetrain()
    def autonomousInit(self):
        pass
    def autonomousPeriodic(self):
        pass
    def teleopInit(self):
        pass
    def teleopPeriodic(self):
        self.drive.drive(0.1,0.1,0.0,self.getPeriod())
    def testInit(self):
        pass
    def testPeriodic(self):
        pass

if __name__ == "__main__":
    wpilib.run(MyRobot)
