import wpimath.units
import math
import wpilib
import wpimath.geometry
import wpimath.kinematics

class ReefNavigationConstants:
    SNAP_RADIUS = wpimath.units.inchesToMeters(23.87490776)

    BLUE_REEF_CENTER_POSITION = wpimath.geometry.Transform2d(
        wpimath.units.inchesToMeters(176.745),
        wpimath.units.inchesToMeters(158.5),
        wpimath.geometry.Rotation2d()
    )

    EAST_TAG_LEFT_POSE_RELATIVE = wpimath.geometry.Pose2d(
        wpimath.units.inchesToMeters(-47.745 - 5), # add like a 5 inch buffer
        wpimath.units.inchesToMeters(6.5 - 5.5), #5.5 is the coral intake offset
        wpimath.geometry.Rotation2d(0.0)
    )

    EAST_TAG_RIGHT_POSE_RELATIVE = wpimath.geometry.Pose2d(
        wpimath.units.inchesToMeters(-47.745 - 5),
        wpimath.units.inchesToMeters(-6.5 - 5.5),
        wpimath.geometry.Rotation2d(0.0)
    )

    BLUE_RIGHT_SETPOINTS = [
        EAST_TAG_RIGHT_POSE_RELATIVE.rotateBy(wpimath.geometry.Rotation2d.fromDegrees(0)).transformBy(BLUE_REEF_CENTER_POSITION),
        EAST_TAG_RIGHT_POSE_RELATIVE.rotateBy(wpimath.geometry.Rotation2d.fromDegrees(60)).transformBy(BLUE_REEF_CENTER_POSITION),
        EAST_TAG_RIGHT_POSE_RELATIVE.rotateBy(wpimath.geometry.Rotation2d.fromDegrees(120)).transformBy(BLUE_REEF_CENTER_POSITION),
        EAST_TAG_RIGHT_POSE_RELATIVE.rotateBy(wpimath.geometry.Rotation2d.fromDegrees(180)).transformBy(BLUE_REEF_CENTER_POSITION),
        EAST_TAG_RIGHT_POSE_RELATIVE.rotateBy(wpimath.geometry.Rotation2d.fromDegrees(240)).transformBy(BLUE_REEF_CENTER_POSITION),
        EAST_TAG_RIGHT_POSE_RELATIVE.rotateBy(wpimath.geometry.Rotation2d.fromDegrees(300)).transformBy(BLUE_REEF_CENTER_POSITION),
    ]

    BLUE_LEFT_SETPOINTS = [
        EAST_TAG_LEFT_POSE_RELATIVE.rotateBy(wpimath.geometry.Rotation2d.fromDegrees(0)).transformBy(BLUE_REEF_CENTER_POSITION),
        EAST_TAG_LEFT_POSE_RELATIVE.rotateBy(wpimath.geometry.Rotation2d.fromDegrees(60)).transformBy(BLUE_REEF_CENTER_POSITION),
        EAST_TAG_LEFT_POSE_RELATIVE.rotateBy(wpimath.geometry.Rotation2d.fromDegrees(120)).transformBy(BLUE_REEF_CENTER_POSITION),
        EAST_TAG_LEFT_POSE_RELATIVE.rotateBy(wpimath.geometry.Rotation2d.fromDegrees(180)).transformBy(BLUE_REEF_CENTER_POSITION),
        EAST_TAG_LEFT_POSE_RELATIVE.rotateBy(wpimath.geometry.Rotation2d.fromDegrees(240)).transformBy(BLUE_REEF_CENTER_POSITION),
        EAST_TAG_LEFT_POSE_RELATIVE.rotateBy(wpimath.geometry.Rotation2d.fromDegrees(300)).transformBy(BLUE_REEF_CENTER_POSITION),
    ]

    BLUE_RIGHT_STATION_CENTER = wpimath.geometry.Pose2d(
        wpimath.units.inchesToMeters(41.73899),
        wpimath.units.inchesToMeters(37.126),
        wpimath.geometry.Rotation2d.fromDegrees(54)
    )

    # BLUE_RIGHT_GROOVE_OFFSET = wpimath.geometry.Transform2d(
    #     wpimath.units.inchesToMeters(8),
    #     wpimath.geometry.Rotation2d.fromDegrees(54-90)
    # )

    # BLUE_RIGHT_STATION_SETPOINTS = [
    #     BLUE_RIGHT_STATION_CENTER,
    #     BLUE_RIGHT_STATION_CENTER.transformBy(BLUE_RIGHT_GROOVE_OFFSET),
    #     BLUE_RIGHT_STATION_CENTER,
    #     BLUE_RIGHT_STATION_CENTER,
    #     BLUE_RIGHT_STATION_CENTER,
    # ]

    BLUE_L1_SETPOINTS = [
        # :>
    ]

    

def getNearestL1Setpoint(robotPos: wpimath.geometry.Pose2d) -> wpimath.geometry.Pose2d:
    robotPos.nearest(ReefNavigationConstants.BLUE_L1_SETPOINTS)
def getNearestLeft(robotPos: wpimath.geometry.Pose2d) -> wpimath.geometry.Pose2d:
    robotPos.nearest(ReefNavigationConstants.BLUE_LEFT_SETPOINTS)
def getNearestRight(robotPos: wpimath.geometry.Pose2d) -> wpimath.geometry.Pose2d:
    robotPos.nearest(ReefNavigationConstants.BLUE_RIGHT_SETPOINTS)