import rev
import wpilib
import enum

class ClimbStates(enum.Enum):
    Raised = enum.auto
    Lowered = enum.auto

# Implemented a dumb state machine even tho zach said not to
# This probably won't work cause im bad- kay
class Climber:
    """
    Initializes a new climber object
    """
    def __init__(self):
        self.climbMotor = rev.SparkMax(17, rev.SparkLowLevel.MotorType.kBrushless)
        self.climbState = ClimbStates.Lowered
        self.timer = wpilib.Timer
        self.CLIMBER_COOLDOWN = 15

    def runClimberRaw(self, amount: float):
        self.climbMotor.set(amount)

    def runClimber(self, desiredState: ClimbStates):
        """
        Climber StateMachine code\n
        Not necessary for use
        """
        if (self.climbState != desiredState):
            if (desiredState == ClimbStates.Raised):
                self.timer.start()
                while self.timer.get() < 12:
                    self.climbMotor.set(0.85)
                self.timer.stop()
            if (desiredState == ClimbStates.Lowered):
                self.timer.start()
                while self.timer.get() < 15:
                    self.climbMotor.set(-0.85)
                self.timer.stop()
                self.climbState == ClimbStates.Lowered
    def setRaised(self):
        self.runClimber(ClimbStates.Raised)
        self.climbState == ClimbStates.Raised
    def setLowered(self):
        self.runClimber(ClimbStates.Lowered)
        self.climbState == ClimbStates.Lowered
        
    def logClimberCurrents(self):
        wpilib.SmartDashboard.putNumber("climb motor current", self.climbMotor.getOutputCurrent())
        wpilib.SmartDashboard.putNumber("climb motor temperature", self.climbMotor.getMotorTemperature())
