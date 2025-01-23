
import math
import enum
import wpilib
import wpimath.kinematics
import wpimath.geometry
import wpimath.controller
import wpimath.trajectory

import rev
from phoenix6 import hardware as ctre
import wpimath.units

class Intake:
    def __init__(self):
        self.coralIntakeMotor = rev.CANSparkMax(0,rev.CANSparkLowLevel.MotorType.kBrushless)
       

        self.WristMotor = rev.CANSparkMax(0,rev.CANSparkLowLevel.MotorType.kBrushless)
        self.WristEncoder = self.WristEncoder.getEncoder()

        self.AlgaeIntakeMotor = rev.CANSparkMax(0,rev.CANSparkLowLevel.MotorType.kBrushless)
        




