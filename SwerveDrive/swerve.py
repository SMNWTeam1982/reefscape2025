import wpilib
import rev._rev as rev
from wpimath.kinematics import SwerveModuleState
from wpimath.geometry import Rotation2d

from phoenix6 import hardware


class SwerveModule:
    def __init__(
            self,
            driveMotorID: int,
            turnMotorID: int
        ):
        self.driveMotor = rev.CANSparkMax(driveMotorID, rev.CANSparkLowLevel.MotorType.kBrushless)
        self.turnMotor = rev.CANSparkMax(turnMotorID, rev.CANSparkLowLevel.MotorType.kBrushless)
        
    def run(self):
        self.driveMotor.set(0.2)
        self.turnMotor.set(0.1)

class DriveTrain():
    pass