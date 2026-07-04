package frc.robot.subsystems.Vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.VisionFusion;
import frc.robot.subsystems.Chassis.CommandSwerveDrivetrain;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;
import org.photonvision.targeting.PhotonTrackedTarget;

/**
 * Collects Limelight MegaTag2 and PhotonVision estimates via {@link VisionIO}, selects one
 * measurement each cycle (hub tags preferred, then MegaTag2, then lowest XY σ), and passes it to
 * WPILib Kalman fusion. Inputs are logged for AdvantageKit replay.
 */
public class VisionSubsystem extends SubsystemBase {

  private final CommandSwerveDrivetrain drive;
  private final VisionIO visionIO;
  private final VisionCameraInputsAutoLogged limelightLeftInputs =
      new VisionCameraInputsAutoLogged();
  private final VisionCameraInputsAutoLogged limelightRightInputs =
      new VisionCameraInputsAutoLogged();
  private final VisionCameraInputsAutoLogged photonBackInputs = new VisionCameraInputsAutoLogged();
  private final VisionCameraInputsAutoLogged photonFrontInputs = new VisionCameraInputsAutoLogged();

  public static VisionSubsystem m_instance;

  public static VisionSubsystem getInstance(CommandSwerveDrivetrain drive) {
    return m_instance == null ? m_instance = new VisionSubsystem(drive) : m_instance;
  }

  public VisionSubsystem(CommandSwerveDrivetrain drive) {
    this.drive = drive;
    if (Constants.currentMode == Constants.Mode.REPLAY) {
      visionIO = new VisionIO() {};
    } else {
      visionIO = new VisionIOLive();
    }
  }

  @Override
  public void periodic() {
    double fpgaNow = Timer.getFPGATimestamp();

    if (!Logger.hasReplaySource()) {
      visionIO.updateInputs(
          limelightLeftInputs,
          limelightRightInputs,
          photonBackInputs,
          photonFrontInputs,
          drive,
          fpgaNow);
    }

    if (!Logger.hasReplaySource()) {
      visionIO.updateInputs(
          limelightLeftInputs,
          limelightRightInputs,
          photonBackInputs,
          photonFrontInputs,
          drive,
          fpgaNow);
    }

    Logger.processInputs("Vision/LimelightLeft", limelightLeftInputs);
    Logger.processInputs("Vision/LimelightRight", limelightRightInputs);
    Logger.processInputs("Vision/PhotonBack", photonBackInputs);
    Logger.processInputs("Vision/PhotonFront", photonFrontInputs);

    List<VisionMeasurement> accepted = new ArrayList<>(4);
    measurementFromInputs(limelightLeftInputs).ifPresent(accepted::add);
    measurementFromInputs(limelightRightInputs).ifPresent(accepted::add);
    measurementFromInputs(photonBackInputs).ifPresent(accepted::add);

    if (accepted.isEmpty()) {
      Logger.recordOutput("VisionFusion/Selected/Accepted", false);
      return;
    }

    VisionMeasurement selected = selectBestMeasurement(accepted);

    drive.addVisionMeasurement(selected.pose(), selected.timestampSeconds(), selected.stdDevs());
    Logger.recordOutput("VisionFusion/Selected/Accepted", true);
    Logger.recordOutput("VisionFusion/Selected/Pose", selected.pose());
    Logger.recordOutput("VisionFusion/Selected/TimestampSeconds", selected.timestampSeconds());
    Logger.recordOutput("VisionFusion/Selected/StdDevX", selected.stdDevs().get(0, 0));
    Logger.recordOutput("VisionFusion/Selected/StdDevY", selected.stdDevs().get(1, 0));
    Logger.recordOutput("VisionFusion/Selected/StdDevYawRad", selected.stdDevs().get(2, 0));
    Logger.recordOutput("VisionFusion/Selected/CandidateCount", accepted.size());
    Logger.recordOutput("VisionFusion/Selected/Source", selected.sourceLabel());
  }

  private static Optional<VisionMeasurement> measurementFromInputs(
      VisionIO.VisionCameraInputs inputs) {
    if (!inputs.hasMeasurement) {
      return Optional.empty();
    }
    return Optional.of(
        new VisionMeasurement(
            inputs.pose,
            inputs.timestampSeconds,
            VecBuilder.fill(
                inputs.stdDevX, inputs.stdDevY, Constants.VisionFusion.PHOTON_THETA_STDDEV_RADIANS),
            MeasurementSource.values()[inputs.measurementSource],
            inputs.sourceLabel,
            tagIdsFromInputs(inputs)));
  }

  private static int[] tagIdsFromInputs(VisionIO.VisionCameraInputs inputs) {
    int[] ids = new int[inputs.tagIdCount];
    if (inputs.tagIdCount > 0) {
      ids[0] = inputs.tagId0;
    }
    if (inputs.tagIdCount > 1) {
      ids[1] = inputs.tagId1;
    }
    if (inputs.tagIdCount > 2) {
      ids[2] = inputs.tagId2;
    }
    if (inputs.tagIdCount > 3) {
      ids[3] = inputs.tagId3;
    }
    return ids;
  }

  /**
   * Prefers hub-tag measurements when any are available; else Limelight MegaTag2; else lowest XY
   * variance among all candidates.
   */
  private static VisionMeasurement selectBestMeasurement(List<VisionMeasurement> measurements) {
    List<VisionMeasurement> hubTag =
        measurements.stream().filter(VisionSubsystem::usesHubTag).toList();
    List<VisionMeasurement> megaTag2 =
        measurements.stream()
            .filter(m -> m.source() == MeasurementSource.LIMELIGHT_MEGATAG2)
            .toList();
    List<VisionMeasurement> pool =
        !hubTag.isEmpty() ? hubTag : megaTag2.isEmpty() ? measurements : megaTag2;
    return pool.stream().min(Comparator.comparingDouble(VisionSubsystem::xyVariance)).orElseThrow();
  }

  private static boolean usesHubTag(VisionMeasurement measurement) {
    for (int tagId : measurement.tagIds()) {
      if (VisionFusion.isHubTag(tagId)) {
        return true;
      }
    }
    return false;
  }

  private static double xyVariance(VisionMeasurement measurement) {
    double sigmaX = measurement.stdDevs().get(0, 0);
    double sigmaY = measurement.stdDevs().get(1, 0);
    return sigmaX * sigmaX + sigmaY * sigmaY;
  }

  enum MeasurementSource {
    LIMELIGHT_MEGATAG2,
    PHOTON
  }

  private record VisionMeasurement(
      Pose2d pose,
      double timestampSeconds,
      Matrix<N3, N1> stdDevs,
      MeasurementSource source,
      String sourceLabel,
      int[] tagIds) {}

  record LlStdDevs(
      double sigmaXMeters, double sigmaYMeters, double sigmaYawRadians, String source) {}

  static Optional<LlStdDevs> parseLimelightNtStdDevs(
      double[] arr, int xIndex, int yIndex, int yawIndex, String source) {
    if (arr.length <= yawIndex) {
      return Optional.empty();
    }
    double sigmaX = arr[xIndex];
    double sigmaY = arr[yIndex];
    double sigmaYawDeg = arr[yawIndex];
    if (!Double.isFinite(sigmaX) || !Double.isFinite(sigmaY) || !Double.isFinite(sigmaYawDeg)) {
      return Optional.empty();
    }
    if (sigmaX <= 0.0 || sigmaY <= 0.0 || sigmaYawDeg <= 0.0) {
      return Optional.empty();
    }
    return Optional.of(new LlStdDevs(sigmaX, sigmaY, Units.degreesToRadians(sigmaYawDeg), source));
  }

  /**
   * Shared rejection gates for all vision sources. {@code ta} is Limelight-style fractional area
   * (0-1) or Photon fractional area ({@code area}/100).
   */
  static boolean shouldReject(
      double ta,
      int tagCount,
      double measurementTimestampSeconds,
      double tagDistanceMeters,
      double fpgaNow,
      ChassisSpeeds chassisSpeeds) {

    if (ta < VisionFusion.REJECT_MIN_TA) {
      return true;
    }
    if (tagCount <= 0) {
      return true;
    }
    if (tagDistanceMeters > VisionFusion.REJECT_MAX_DISTANCE_METERS) {
      return true;
    }
    if (fpgaNow - measurementTimestampSeconds > VisionFusion.REJECT_STALE_SECONDS) {
      return true;
    }
    if (VisionFusion.REJECT_ON_HIGH_OMEGA
        && Math.abs(chassisSpeeds.omegaRadiansPerSecond)
            > VisionFusion.REJECT_MAX_OMEGA_RAD_PER_SEC) {
      return true;
    }
    return false;
  }

  static double limelightTagDistanceToRobotMeters(LimelightHelpers.PoseEstimate mt2) {
    if (mt2.rawFiducials != null && mt2.rawFiducials.length > 0) {
      double sum = 0.0;
      for (LimelightHelpers.RawFiducial f : mt2.rawFiducials) {
        sum += f.distToRobot;
      }
      return sum / mt2.rawFiducials.length;
    }
    return mt2.avgTagDist;
  }

  static int[] limelightTagIds(LimelightHelpers.PoseEstimate mt2) {
    if (mt2.rawFiducials == null || mt2.rawFiducials.length == 0) {
      return new int[0];
    }
    int[] ids = new int[mt2.rawFiducials.length];
    for (int i = 0; i < mt2.rawFiducials.length; i++) {
      ids[i] = mt2.rawFiducials[i].id;
    }
    return ids;
  }

  static int[] photonTagIds(List<PhotonTrackedTarget> targets) {
    return targets.stream().mapToInt(PhotonTrackedTarget::getFiducialId).toArray();
  }

  static double photonAvgAreaFraction(List<PhotonTrackedTarget> targets) {
    if (targets.isEmpty()) {
      return 0.0;
    }
    double sum = 0.0;
    for (PhotonTrackedTarget t : targets) {
      sum += t.area / 100.0;
    }
    return sum / targets.size();
  }

  static double photonAvgCameraToTagDistanceMeters(List<PhotonTrackedTarget> targets) {
    if (targets.isEmpty()) {
      return 0.0;
    }
    double sum = 0.0;
    for (PhotonTrackedTarget t : targets) {
      sum += photonCameraToTagDistanceMeters(t);
    }
    return sum / targets.size();
  }

  static double photonCameraToTagDistanceMeters(PhotonTrackedTarget t) {
    return t.getBestCameraToTarget().getTranslation().getNorm();
  }
}
