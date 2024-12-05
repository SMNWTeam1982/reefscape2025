import wpilib
import wpimath
import rev._rev as rev
import wpimath.kinematics
from SwerveDrive import SwerveModule
from wpimath.geometry import Rotation2d


class MyRobot(wpilib.TimedRobot):
    def robotInit(self):
        self.frontRight = SwerveModule.SwerveModule(5,6,1)
    
    def autonomousInit(self):
        pass

    def autonomousPeriodic(self):
        pass
    def teleopInit(self):
        pass
    def teleopPeriodic(self):
        self.frontRight.run(wpimath.kinematics.SwerveModuleState(0,Rotation2d(0)))        
    def testInit(self):
        pass
    def testPeriodic(self):
        pass

if __name__ == "__main__":
    wpilib.run(MyRobot)