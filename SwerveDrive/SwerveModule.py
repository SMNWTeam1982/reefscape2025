import wpilib
import rev._rev as rev
from wpimath.kinematics import SwerveModuleState
from wpimath.geometry import Rotation2d
from wpimath.controller import PIDController

from phoenix6 import hardware as ctre

# these is calculated through testing, DO NOT CHANGE, these should be the same for all swerve modules, slight differences will be averaged out
class ModuleConstants: 
    # kA can be ignored because as of 2024 all approved frc motors have this: 
    # "the relationship between voltage and acceleration (at constant velocity) is almost perfectly linear for FRC components" - wpilib docs
    # position motor set point = kS * sign( desired velocity ) + kV * (desired velocity) + TODO

    # also called kS (voltage needed to overcome static friction)
    # a kS of 0 means that no matter how small of a value you give to the motor it will move
    # kS of 0 is not possible in real life, but kS can be set to 0 if you want to ignore it
    DRIVE_MOTOR_MINIMUM_SET_BEFORE_MOVEMENT = 0.0 # [untuned]
    TURN_MOTOR_MINIMUM_SET_BEFORE_MOVEMENT = 0.0 # [untuned]

    # also called kV (variable friction, increases with voltage) 
    # also called f because kV is a feedforward value
    # kV of 0 means that the motor experiences no friction
    # kV of 0 means set it to 0.5 then 0.0 the motor will stay at 0.5 speed
    # kV of 0 is not possible in real life
    # kV is only necessary for velocity controlers
    DRIVE_MOTOR_SET_NEEDED_TO_HOLD_VELOCITY = 0.0 # [untuned]

    # also called kP
    # error * kP
    DRIVE_MOTOR_PROPORTIONAL_GAIN = 0.0 # [untuned]

    # also called kI
    # accumulatedError * kI
    DRIVE_MOTOR_INTEGRAL_GAIN = 0.0 # [untuned]

    # also called kD
    # rateError * kD
    DRIVE_MOTOR_DERIVATIVE_GAIN = 0.0 # [untuned]

    # self explanitory (i hope)
    DRIVE_MOTOR_MAX_VELOCITY_METERS_PER_SECOND = 1.0 # [untuned]


    # these coefficients must be positive
    TURN_MOTOR_PROPORTIONAL_COEFFICIENT = 0.0 # [untuned]
    TURN_MOTOR_INTEGRAL_COEFFICIENT = 0.0 # [untuned]
    TURN_MOTOR_DERIVATIVE_COEFFICIENT = 0.0 # [untuned]
    #TURN_MOTOR_FEEDFOREWARD = 0.0 # [untuned]



class SwerveModule:
    def __init__(
        self,
        driveMotorID: int,
        turnMotorID: int,
        turnEncoderID: int
    ):
        self.driveMotor = rev.CANSparkMax(driveMotorID, rev.CANSparkLowLevel.MotorType.kBrushless)
        self.turnMotor = rev.CANSparkMax(turnMotorID, rev.CANSparkLowLevel.MotorType.kBrushless)
        self.turnEncoder = ctre.CANcoder(turnEncoderID)
        self.turnPID = PIDController(
            ModuleConstants.TURN_MOTOR_PROPORTIONAL_COEFFICIENT,
            ModuleConstants.TURN_MOTOR_INTEGRAL_COEFFICIENT,
            ModuleConstants.TURN_MOTOR_DERIVATIVE_COEFFICIENT
        )

        

    def getRotation(self) -> Rotation2d:
        return Rotation2d.fromRotations( self.turnEncoder.get_position() ) # relative position
        
    def run(self, unoptimezedDesiredState: SwerveModuleState):
        currentAngle = self.getRotation()

        OptimizedState = SwerveModuleState.optimize(unoptimezedDesiredState, currentAngle)
        OptimizedState.speed *= (OptimizedState.angle - currentAngle).cos()