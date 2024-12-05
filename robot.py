# TODO: insert robot code here
import wpilib
import rev._rev as rev


class MyRobot(wpilib.TimedRobot):
    def robotInit(self):
        self.leftDrive = rev.CANSparkMax(1, rev.CANSparkLowLevel.MotorType.kBrushless)
        self.controller = wpilib.XboxController(0)
        self.timer = wpilib.Timer()
    
    def autonomousInit(self):
        self.timer.restart()

    def autonomousPeriodic(self):
        if self.timer.get() < 2.0:
            self.leftDrive.set(0.5)
        else:
            self.leftDrive.stopMotor()
    
    def teleopInit(self):
        pass
    def teleopPeriodic(self):
        self.leftDrive.set(self.controller.getLeftY())
        
    def testInit(self):
        pass
    def testPeriodic(self):
        pass

if __name__ == "__main__":
    wpilib.run(MyRobot)