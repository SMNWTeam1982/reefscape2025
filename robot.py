import wpilib
import wpimath
import rev._rev as rev
import wpimath.kinematics
from SwerveDrive import Drive
from wpimath.geometry import Rotation2d


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
        drive.drive(0.1,0.1,0.0)
    def testInit(self):
        pass
    def testPeriodic(self):
        pass

if __name__ == "__main__":
    wpilib.run(MyRobot)
