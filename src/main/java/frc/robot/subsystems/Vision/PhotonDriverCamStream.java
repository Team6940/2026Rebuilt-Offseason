package frc.robot.subsystems.Vision;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants.DriverCam;
import org.littletonrobotics.junction.Logger;

/** Republishes PhotonFront driver-mode MJPEG to CameraServer for dashboard viewing. */
public final class PhotonDriverCamStream {

  private PhotonDriverCamStream() {}

  /** Binds HttpCamera + optional USB-tether port forwards. Call once from robot init on real hardware. */
  public static void start() {
    String host = resolveCoprocessorHost();
    int outputPort = DriverCam.PHOTON_FRONT_OUTPUT_STREAM_PORT;
    String streamUrl = "http://" + host + ":" + outputPort + "/?action=stream";

    if (DriverCam.ENABLE_PORT_FORWARDING) {
      forwardPhotonPorts(host, outputPort);
    }

    HttpCamera camera = new HttpCamera(DriverCam.STREAM_NAME, streamUrl);
    camera.setConnectionStrategy(HttpCamera.ConnectionStrategy.kKeepOpen);
    CameraServer.startAutomaticCapture(camera);

    Logger.recordOutput("Vision/DriverCam/Host", host);
    Logger.recordOutput("Vision/DriverCam/StreamUrl", streamUrl);
    Logger.recordOutput("Vision/DriverCam/Started", true);
  }

  private static String resolveCoprocessorHost() {
    if (!DriverCam.PHOTON_COPROCESSOR_STATIC_IP.isEmpty()) {
      return DriverCam.PHOTON_COPROCESSOR_STATIC_IP;
    }

    int team = RobotController.getTeamNumber();
    if (team <= 0) {
      team = DriverCam.DEFAULT_TEAM_NUMBER;
    }
    return String.format(
        "10.%d.%d.%d",
        team / 100, team % 100, DriverCam.PHOTON_COPROCESSOR_HOST_OCTET);
  }

  private static void forwardPhotonPorts(String host, int outputPort) {
    PortForwarder.add(DriverCam.PHOTON_WEB_UI_PORT, host, DriverCam.PHOTON_WEB_UI_PORT);
    PortForwarder.add(outputPort, host, outputPort);
    PortForwarder.add(outputPort - 1, host, outputPort - 1);
  }
}
