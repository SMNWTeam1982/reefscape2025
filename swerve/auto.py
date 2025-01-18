
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
