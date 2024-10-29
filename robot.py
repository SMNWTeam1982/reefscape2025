import wpilib
import rev._rev as rev
from SwerveDrive import swerve


class MyRobot(wpilib.TimedRobot):
    def robotInit(self):
        self.frontRight = swerve.SwerveModule(3,2)
    
    def autonomousInit(self):
        pass

    def autonomousPeriodic(self):
        pass
    def teleopInit(self):
        pass
    def telopPeriodic(self):
        self.frontRight.run()
        
    def testInit(self):
        pass
    def testPeriodic(self):
        pass

if __name__ == "__main__":
    wpilib.run(MyRobot)