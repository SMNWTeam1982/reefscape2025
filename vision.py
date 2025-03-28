from cscore import CameraServer
def main():
    CameraServer.enableLogging()
    elevatorCamera = CameraServer.startAutomaticCapture(0)
    climberCamera = CameraServer.startAutomaticCapture(1)
    elevatorCamera.setFPS(30)
    climberCamera.setFPS(30)
    elevatorCamera.setResolution(400, 400)
    climberCamera.setResolution(400, 400)
    CameraServer.waitForever()