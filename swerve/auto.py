from pathplannerlib import PathPlanner

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

def getRobotRelativeSpeeds(self):
        # Assuming the path is already loaded and you are following it
        # Get the current position along the path (in terms of distance traveled or time)
        current_time = self.getCurrentTime()
        state = self.path.getStateAtTime(current_time)

        # This state provides the desired velocity (vx, vy, omega) in field-relative coordinates
        field_relative_speeds = state.velocity

        # Convert to robot-relative speeds
        robot_relative_speeds = self.fieldToRobotRelative(field_relative_speeds)

        return robot_relative_speeds


  
