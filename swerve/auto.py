
class Auto:
  def __init__():
    self.state = "idle"

  def run(measurment: float):
    match self.state:
      case "idle":
        if measurement < 1.0
          self.state = "running"
      case "running":
        if measurement > 1.0:
          self.state = "idle"

def fieldToRobotRelative(self, field_speeds):

        robot_angle = self.getPose().rotation().getRadians()


        vx_robot = field_speeds.vx * math.cos(robot_angle) + field_speeds.vy * math.sin(robot_angle)
        vy_robot = -field_speeds.vx * math.sin(robot_angle) + field_speeds.vy * math.cos(robot_angle)
        omega_robot = field_speeds.omega  # Angular velocity remains the same

        return ChassisSpeeds(vx_robot, vy_robot, omega_robot)
  
