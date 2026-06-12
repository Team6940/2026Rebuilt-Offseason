package frc.robot.subsystems.Vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.PoseEstimatorConstants;
import frc.robot.Constants.VisionFusion;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Chassis.CommandSwerveDrivetrain;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

/**
 * Collects Limelight MegaTag2 and PhotonVision estimates, selects one measurement each cycle
 * (MegaTag2 preferred, then lowest XY σ), and passes it to WPILib Kalman fusion.
 */
public class VisionSubsystem extends SubsystemBase {

  private static final double[] LL_DEFAULT_STDDEVS =
      new double[VisionFusion.LL_STDDEV_ARRAY_LENGTH];

  private final CommandSwerveDrivetrain drive;
  private final PhotonCameraSetup photonLeft;
  private final PhotonCameraSetup photonRight;
  public static VisionSubsystem m_instance;

  private final NetworkTableEntry llLeftSnapshot =
      NetworkTableInstance.getDefault().getTable(RobotContainer.limelightLeft).getEntry("snapshot");
  private final NetworkTableEntry llRightSnapshot =
      NetworkTableInstance.getDefault()
          .getTable(RobotContainer.limelightRight)
          .getEntry("snapshot");

  public static VisionSubsystem getInstance(CommandSwerveDrivetrain drive) {
    return m_instance == null ? m_instance = new VisionSubsystem(drive) : m_instance;
  }

  public VisionSubsystem(CommandSwerveDrivetrain drive) {
    this.drive = drive;
    this.photonLeft =
        new PhotonCameraSetup(
            RobotContainer.photonCameraLeft, "Left", VisionFusion.kRobotToPhotonL);
    this.photonRight =
        new PhotonCameraSetup(
            RobotContainer.photonCameraRight, "Right", VisionFusion.kRobotToPhotonR);
  }

  @Override
  public void periodic() {
    double fpgaNow = Timer.getFPGATimestamp();

    llLeftSnapshot.setNumber(0);
    llRightSnapshot.setNumber(0);

    List<VisionMeasurement> accepted = new ArrayList<>(4);
    processLimelight(RobotContainer.limelightLeft, "Left", fpgaNow).ifPresent(accepted::add);
    processLimelight(RobotContainer.limelightRight, "Right", fpgaNow).ifPresent(accepted::add);
    processPhoton(photonLeft, fpgaNow).ifPresent(accepted::add);
    processPhoton(photonRight, fpgaNow).ifPresent(accepted::add);

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

  /**
   * Prefers Limelight MegaTag2 when available; otherwise lowest XY variance among all candidates.
   */
  private static VisionMeasurement selectBestMeasurement(List<VisionMeasurement> measurements) {
    List<VisionMeasurement> megaTag2 =
        measurements.stream()
            .filter(m -> m.source() == MeasurementSource.LIMELIGHT_MEGATAG2)
            .toList();
    List<VisionMeasurement> pool = megaTag2.isEmpty() ? measurements : megaTag2;
    return pool.stream().min(Comparator.comparingDouble(VisionSubsystem::xyVariance)).orElseThrow();
  }

  private static double xyVariance(VisionMeasurement measurement) {
    double sigmaX = measurement.stdDevs().get(0, 0);
    double sigmaY = measurement.stdDevs().get(1, 0);
    return sigmaX * sigmaX + sigmaY * sigmaY;
  }

  private Optional<VisionMeasurement> processLimelight(
      String limelightName, String logSide, double fpgaNow) {
    drive.applyLimelightGyroForMegaTag2(limelightName);

    LimelightHelpers.PoseEstimate mt2 =
        LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName);

    logLimelightPoseEstimate(logSide, mt2);

    if (mt2 == null) {
      Logger.recordOutput("VisionFusion/Limelight/" + limelightName + "/Connected", false);
      return Optional.empty();
    }

    Logger.recordOutput("VisionFusion/Limelight/" + limelightName + "/Connected", true);

    if (!LimelightHelpers.getTV(limelightName)) {
      Logger.recordOutput("VisionFusion/Limelight/" + limelightName + "/SeesTarget", false);
      Logger.recordOutput("VisionFusion/Limelight/" + limelightName + "/Accepted", false);
      return Optional.empty();
    }
    Logger.recordOutput("VisionFusion/Limelight/" + limelightName + "/SeesTarget", true);

    if (mt2.tagCount <= 0) {
      Logger.recordOutput("VisionFusion/Limelight/" + limelightName + "/Accepted", false);
      return Optional.empty();
    }

    NetworkTableEntry snapshotEntry =
        limelightName.equals(RobotContainer.limelightLeft) ? llLeftSnapshot : llRightSnapshot;
    snapshotEntry.setNumber(1);

    ChassisSpeeds speeds = drive.getChassisSpeeds();

    double tagDistRobotM = limelightTagDistanceToRobotMeters(mt2);

    if (shouldReject(
        mt2.avgTagArea, mt2.tagCount, mt2.timestampSeconds, tagDistRobotM, fpgaNow, speeds)) {
      Logger.recordOutput("VisionFusion/Limelight/" + limelightName + "/Accepted", false);
      return Optional.empty();
    }

    LlStdDevs std = resolveLimelightStdDevs(limelightName, mt2.avgTagArea);
    Logger.recordOutput("VisionFusion/Limelight/" + limelightName + "/StdDevsValid", true);
    Logger.recordOutput("VisionFusion/Limelight/" + limelightName + "/StdDevX", std.sigmaXMeters());
    Logger.recordOutput("VisionFusion/Limelight/" + limelightName + "/StdDevY", std.sigmaYMeters());
    Logger.recordOutput(
        "VisionFusion/Limelight/" + limelightName + "/StdDevYawRad", std.sigmaYawRadians());
    Logger.recordOutput("VisionFusion/Limelight/" + limelightName + "/StdDevSource", std.source());

    Logger.recordOutput("VisionFusion/Limelight/" + limelightName + "/Accepted", true);
    return Optional.of(
        new VisionMeasurement(
            mt2.pose,
            mt2.timestampSeconds,
            VecBuilder.fill(std.sigmaXMeters(), std.sigmaYMeters(), std.sigmaYawRadians()),
            MeasurementSource.LIMELIGHT_MEGATAG2,
            "Limelight" + logSide));
  }

  private Optional<VisionMeasurement> processPhoton(PhotonCameraSetup setup, double fpgaNow) {
    String cameraName = setup.camera.getName();
    setup.poseEstimator.addHeadingData(fpgaNow, drive.getPose().getRotation());

    List<PhotonPipelineResult> unread = setup.camera.getAllUnreadResults();
    PhotonPipelineResult result =
        unread.stream()
            .filter(PhotonPipelineResult::hasTargets)
            .max(Comparator.comparingDouble(PhotonPipelineResult::getTimestampSeconds))
            .orElse(null);
    if (result == null) {
      logPhotonPoseEstimate(setup.logSide, Optional.empty(), Optional.empty());
      Logger.recordOutput(
          "VisionFusion/Photon/" + cameraName + "/Connected", setup.camera.isConnected());
      Logger.recordOutput("VisionFusion/Photon/" + cameraName + "/SeesTarget", false);
      Logger.recordOutput("VisionFusion/Photon/" + cameraName + "/Accepted", false);
      return Optional.empty();
    }

    Logger.recordOutput(
        "VisionFusion/Photon/" + cameraName + "/Connected", setup.camera.isConnected());
    Logger.recordOutput("VisionFusion/Photon/" + cameraName + "/SeesTarget", true);

    Optional<EstimatedRobotPose> multiOpt = setup.poseEstimator.estimateCoprocMultiTagPose(result);
    Optional<EstimatedRobotPose> pnpOpt =
        multiOpt.isPresent()
            ? Optional.empty()
            : setup.poseEstimator.estimatePnpDistanceTrigSolvePose(result);
    logPhotonPoseEstimate(setup.logSide, multiOpt, pnpOpt);

    if (multiOpt.isPresent()) {
      EstimatedRobotPose est = multiOpt.get();
      Pose2d pose = est.estimatedPose.toPose2d();
      List<PhotonTrackedTarget> targets = est.targetsUsed;
      int tagCount = Math.max(1, targets.size());
      double taMean = photonAvgAreaFraction(targets);
      double tagDistM = photonAvgCameraToTagDistanceMeters(targets);

      if (shouldReject(
          taMean, tagCount, est.timestampSeconds, tagDistM, fpgaNow, drive.getChassisSpeeds())) {
        Logger.recordOutput("VisionFusion/Photon/" + cameraName + "/Accepted", false);
        return Optional.empty();
      }

      double xyStd = PoseEstimatorConstants.tAtoDev.get(taMean);
      Logger.recordOutput("VisionFusion/Photon/" + cameraName + "/Accepted", true);
      return Optional.of(
          new VisionMeasurement(
              pose,
              est.timestampSeconds,
              VecBuilder.fill(xyStd, xyStd, VisionFusion.PHOTON_THETA_STDDEV_RADIANS),
              MeasurementSource.PHOTON,
              "Photon" + setup.logSide));
    }

    if (pnpOpt.isEmpty()) {
      Logger.recordOutput("VisionFusion/Photon/" + cameraName + "/Accepted", false);
      return Optional.empty();
    }

    EstimatedRobotPose est = pnpOpt.get();
    Pose2d pose = est.estimatedPose.toPose2d();
    PhotonTrackedTarget best = result.getBestTarget();
    if (best == null) {
      Logger.recordOutput("VisionFusion/Photon/" + cameraName + "/Accepted", false);
      return Optional.empty();
    }

    double taFrac = best.area / 100.0;
    double tagDistM = photonCameraToTagDistanceMeters(best);

    if (shouldReject(
        taFrac, 1, est.timestampSeconds, tagDistM, fpgaNow, drive.getChassisSpeeds())) {
      Logger.recordOutput("VisionFusion/Photon/" + cameraName + "/Accepted", false);
      return Optional.empty();
    }

    double xyStd = PoseEstimatorConstants.tAtoDev.get(taFrac);
    Logger.recordOutput("VisionFusion/Photon/" + cameraName + "/Accepted", true);
    return Optional.of(
        new VisionMeasurement(
            pose,
            est.timestampSeconds,
            VecBuilder.fill(xyStd, xyStd, VisionFusion.PHOTON_THETA_STDDEV_RADIANS),
            MeasurementSource.PHOTON,
            "Photon" + setup.logSide));
  }

  /**
   * Logs MegaTag2 {@link LimelightHelpers.PoseEstimate} data for AdvantageScope (pose + metadata).
   */
  private static void logLimelightPoseEstimate(String side, LimelightHelpers.PoseEstimate mt2) {
    String base = "VisionFusion/PoseEstimates/Limelight" + side;
    if (mt2 == null) {
      Logger.recordOutput(base, new Pose2d());
      Logger.recordOutput(base + "/HasData", false);
      return;
    }
    Logger.recordOutput(base, mt2.pose);
    Logger.recordOutput(base + "/HasData", mt2.tagCount > 0);
    Logger.recordOutput(base + "/TimestampSeconds", mt2.timestampSeconds);
    Logger.recordOutput(base + "/Latency", mt2.latency);
    Logger.recordOutput(base + "/TagCount", mt2.tagCount);
    Logger.recordOutput(base + "/TagSpan", mt2.tagSpan);
    Logger.recordOutput(base + "/AvgTagDist", mt2.avgTagDist);
    Logger.recordOutput(base + "/AvgTagArea", mt2.avgTagArea);
    Logger.recordOutput(base + "/MegaTag2", mt2.isMegaTag2);
  }

  /**
   * Limelight MegaTag2 σ published on NT {@code stddevs}: {@code [MT1…, MT2x, MT2y, MT2z, MT2roll,
   * MT2pitch, MT2yaw]}.
   *
   * <p>Priority: MT2 indices → MT1 indices → {@link PoseEstimatorConstants#tAtoDev} from {@code
   * avgTagArea}. Lookup fallback uses a very large θ σ so fusion weights gyro for heading.
   */
  private static LlStdDevs resolveLimelightStdDevs(String limelightName, double avgTagArea) {
    double[] arr =
        LimelightHelpers.getLimelightNTTableEntry(limelightName, "stddevs")
            .getDoubleArray(LL_DEFAULT_STDDEVS);

    Optional<LlStdDevs> mt2 =
        parseLimelightNtStdDevs(
            arr,
            VisionFusion.LL_MT2_X_STDDEV_INDEX,
            VisionFusion.LL_MT2_Y_STDDEV_INDEX,
            VisionFusion.LL_MT2_YAW_STDDEV_INDEX,
            "MT2");
    if (mt2.isPresent()) {
      return mt2.get();
    }

    Optional<LlStdDevs> mt1 =
        parseLimelightNtStdDevs(
            arr,
            VisionFusion.LL_MT1_X_STDDEV_INDEX,
            VisionFusion.LL_MT1_Y_STDDEV_INDEX,
            VisionFusion.LL_MT1_YAW_STDDEV_INDEX,
            "MT1");
    if (mt1.isPresent()) {
      return mt1.get();
    }

    double taFrac = avgTagArea / 100.0;
    double xyStd = PoseEstimatorConstants.tAtoDev.get(taFrac);
    return new LlStdDevs(xyStd, xyStd, VisionFusion.PHOTON_THETA_STDDEV_RADIANS, "TA_LOOKUP");
  }

  private static Optional<LlStdDevs> parseLimelightNtStdDevs(
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

  private enum MeasurementSource {
    LIMELIGHT_MEGATAG2,
    PHOTON
  }

  private record VisionMeasurement(
      Pose2d pose,
      double timestampSeconds,
      Matrix<N3, N1> stdDevs,
      MeasurementSource source,
      String sourceLabel) {}

  private record LlStdDevs(
      double sigmaXMeters, double sigmaYMeters, double sigmaYawRadians, String source) {}

  private static final class PhotonCameraSetup {
    final PhotonCamera camera;
    final PhotonPoseEstimator poseEstimator;
    final String logSide;

    PhotonCameraSetup(String cameraName, String logSide, Transform3d robotToCamera) {
      this.camera = new PhotonCamera(cameraName);
      this.poseEstimator =
          new PhotonPoseEstimator(Constants.FieldConstants.getAprilTagFieldLayout(), robotToCamera);
      this.logSide = logSide;
    }
  }

  /**
   * Logs Photon estimated pose (multi-tag preferred, else PNP distance trig) for AdvantageScope.
   */
  private static void logPhotonPoseEstimate(
      String side, Optional<EstimatedRobotPose> multiOpt, Optional<EstimatedRobotPose> pnpOpt) {
    String base = "VisionFusion/PoseEstimates/Photon" + side;
    if (multiOpt.isPresent()) {
      EstimatedRobotPose est = multiOpt.get();
      Logger.recordOutput(base, est.estimatedPose.toPose2d());
      Logger.recordOutput(base + "/HasData", true);
      Logger.recordOutput(base + "/TimestampSeconds", est.timestampSeconds);
      Logger.recordOutput(base + "/Strategy", "MULTI_TAG_PNP");
      Logger.recordOutput(base + "/TagCount", est.targetsUsed.size());
      return;
    }
    if (pnpOpt.isPresent()) {
      EstimatedRobotPose est = pnpOpt.get();
      Logger.recordOutput(base, est.estimatedPose.toPose2d());
      Logger.recordOutput(base + "/HasData", true);
      Logger.recordOutput(base + "/TimestampSeconds", est.timestampSeconds);
      Logger.recordOutput(base + "/Strategy", "SINGLE_TAG_PNP_DISTANCE_TRIG");
      Logger.recordOutput(base + "/TagCount", est.targetsUsed.size());
      return;
    }
    Logger.recordOutput(base, new Pose2d());
    Logger.recordOutput(base + "/HasData", false);
    Logger.recordOutput(base + "/Strategy", "none");
  }

  private static double photonAvgAreaFraction(List<PhotonTrackedTarget> targets) {
    if (targets.isEmpty()) {
      return 0.0;
    }
    double sum = 0.0;
    for (PhotonTrackedTarget t : targets) {
      sum += t.area / 100.0;
    }
    return sum / targets.size();
  }

  /**
   * Shared rejection gates for all vision sources. {@code ta} is Limelight-style fractional area
   * (0–1) or Photon fractional area ({@code area}/100).
   *
   * <p>{@code tagDistanceMeters}: Limelight uses mean {@link
   * LimelightHelpers.RawFiducial#distToRobot} when fiducials exist; otherwise {@link
   * LimelightHelpers.PoseEstimate#avgTagDist}. Photon uses mean camera–tag range from {@code
   * getBestCameraToTarget()} (meters).
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

  /**
   * Tag–robot distance from Limelight: arithmetic mean {@link
   * LimelightHelpers.RawFiducial#distToRobot}; if fiducials are missing uses {@link
   * LimelightHelpers.PoseEstimate#avgTagDist} (meters).
   */
  private static double limelightTagDistanceToRobotMeters(LimelightHelpers.PoseEstimate mt2) {
    if (mt2.rawFiducials != null && mt2.rawFiducials.length > 0) {
      double sum = 0.0;
      for (LimelightHelpers.RawFiducial f : mt2.rawFiducials) {
        sum += f.distToRobot;
      }
      return sum / mt2.rawFiducials.length;
    }
    return mt2.avgTagDist;
  }

  /** Mean camera–tag range (solve translation norm, meters) over visible targets. */
  private static double photonAvgCameraToTagDistanceMeters(List<PhotonTrackedTarget> targets) {
    if (targets.isEmpty()) {
      return 0.0;
    }
    double sum = 0.0;
    for (PhotonTrackedTarget t : targets) {
      sum += photonCameraToTagDistanceMeters(t);
    }
    return sum / targets.size();
  }

  private static double photonCameraToTagDistanceMeters(PhotonTrackedTarget t) {
    return t.getBestCameraToTarget().getTranslation().getNorm();
  }

  private static double maxAbsPhotonYawRad(List<PhotonTrackedTarget> targets) {
    double maxDeg = 0.0;
    for (PhotonTrackedTarget t : targets) {
      maxDeg = Math.max(maxDeg, Math.abs(t.yaw));
    }
    return Units.degreesToRadians(maxDeg);
  }
}
