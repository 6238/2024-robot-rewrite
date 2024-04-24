package frc.robot.util.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.telemetry.Alert;
import java.util.Optional;
import org.photonvision.*;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

/**
 * Utility class to work with reading from a camera attached via PhotonVision and feeding the data
 * into a {@link org.photonvision.PhotonPoseEstimator}.
 */
public class PopcornCamera {
  private PhotonCamera cam;
  private PhotonPoseEstimator poseEst;

  private Alert camDisconnected;

  /**
   * Creates a new PopcornCamera.
   *
   * @param camName The name of the camera - this must match what's set in the PhotonVision
   *     settings.
   * @param robotToCam An {@link edu.wpi.first.math.geometry.Transform3d} representing the distance
   *     and rotation from the robot's center to the camera's lens.
   * @param layout The layout of the field.
   */
  public PopcornCamera(String camName, Transform3d robotToCam, AprilTagFieldLayout layout) {
    cam = new PhotonCamera(camName);
    poseEst = new PhotonPoseEstimator(layout, PoseStrategy.AVERAGE_BEST_TARGETS, cam, robotToCam);

    camDisconnected = new Alert("Camera " + camName + " is disconnected!", Alert.AlertType.ERROR);
  }

  public Optional<EstimatedRobotPose> update() {
    if (!cam.isConnected()) {
      camDisconnected.set(true);
      return Optional.empty();
    } else {
      camDisconnected.set(false);
      Optional<EstimatedRobotPose> pose = poseEst.update();
      if (pose.isPresent()) {
        return pose;
      } else {
        return Optional.empty();
      }
    }
  }
}
