// Copyright 2021-2025 FRC 6328 / team extensions
//
// Fuses multiple independent vision pose measurements into {@link
// edu.wpi.first.math.estimator.SwerveDrivePoseEstimator} using per-measurement standard deviations
// (no manual pose averaging).

package frc.robot.subsystems.Vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
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
 * Collects Limelight MegaTag2 (σ from NT {@code stddevs} with TA lookup fallback; see {@link
 * #resolveLimelightStdDevs(String, double)}) and PhotonVision estimates. Photon translation σ uses {@link PoseEstimatorConstants#tAtoDev} from
 * mean target area fraction ({@code area}/100).
 */
public class VisionSubsystem extends SubsystemBase {

  private static final double[] LL_DEFAULT_STDDEVS =
      new double[VisionFusion.LL_STDDEV_ARRAY_LENGTH];

  private final CommandSwerveDrivetrain drive;
  private final PhotonCamera photonCamera;
  private final PhotonPoseEstimator photonPoseEstimator;
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
    this.photonCamera = new PhotonCamera(RobotContainer.photonCameraName);
    this.photonPoseEstimator =
        new PhotonPoseEstimator(
            Constants.FieldConstants.getAprilTagFieldLayout(), VisionFusion.kRobotToCamera);
  }

  @Override
  public void periodic() {
    double fpgaNow = Timer.getFPGATimestamp();

    llLeftSnapshot.setNumber(0);
    llRightSnapshot.setNumber(0);

    List<VisionMeasurement> accepted = new ArrayList<>(3);
    processLimelight(RobotContainer.limelightLeft, "Left", fpgaNow).ifPresent(accepted::add);
    processLimelight(RobotContainer.limelightRight, "Right", fpgaNow).ifPresent(accepted::add);
    processPhoton(fpgaNow).ifPresent(accepted::add);

    if (accepted.isEmpty()) {
      Logger.recordOutput("VisionFusion/Fused/Accepted", false);
      return;
    }

    VisionMeasurement fused = accepted.get(0);
    for (int i = 1; i < accepted.size(); i++) {
      fused = fuseMeasurements(fused, accepted.get(i));
    }

    drive.addVisionMeasurement(
        fused.pose(), fused.timestampSeconds(), fused.stdDevs());
    Logger.recordOutput("VisionFusion/Fused/Accepted", true);
    Logger.recordOutput("VisionFusion/Fused/Pose", fused.pose());
    Logger.recordOutput("VisionFusion/Fused/TimestampSeconds", fused.timestampSeconds());
    Logger.recordOutput("VisionFusion/Fused/StdDevX", fused.stdDevs().get(0, 0));
    Logger.recordOutput("VisionFusion/Fused/StdDevY", fused.stdDevs().get(1, 0));
    Logger.recordOutput("VisionFusion/Fused/StdDevYawRad", fused.stdDevs().get(2, 0));
    Logger.recordOutput("VisionFusion/Fused/SourceCount", accepted.size());
  }

  /** Fuses two vision measurements with inverse-variance weighting (newer timestamp wins). */
  private VisionMeasurement fuseMeasurements(VisionMeasurement a, VisionMeasurement b) {
    if (b.timestampSeconds() < a.timestampSeconds()) {
      VisionMeasurement tmp = a;
      a = b;
      b = tmp;
    }

    Optional<Pose2d> poseAtA = drive.samplePoseAt(a.timestampSeconds());
    Optional<Pose2d> poseAtB = drive.samplePoseAt(b.timestampSeconds());
    if (poseAtA.isEmpty() || poseAtB.isEmpty()) {
      return b;
    }

    Transform2d a_T_b = poseAtB.get().minus(poseAtA.get());
    Pose2d poseA = a.pose().transformBy(a_T_b);
    Pose2d poseB = b.pose();

    Matrix<N3, N1> varianceA = a.stdDevs().elementTimes(a.stdDevs());
    Matrix<N3, N1> varianceB = b.stdDevs().elementTimes(b.stdDevs());

    Rotation2d fusedHeading = poseB.getRotation();
    if (varianceA.get(2, 0) < VisionFusion.LARGE_VARIANCE
        && varianceB.get(2, 0) < VisionFusion.LARGE_VARIANCE) {
      fusedHeading =
          new Rotation2d(
              poseA.getRotation().getCos() / varianceA.get(2, 0)
                  + poseB.getRotation().getCos() / varianceB.get(2, 0),
              poseA.getRotation().getSin() / varianceA.get(2, 0)
                  + poseB.getRotation().getSin() / varianceB.get(2, 0));
    }

    double weightAx = 1.0 / varianceA.get(0, 0);
    double weightAy = 1.0 / varianceA.get(1, 0);
    double weightBx = 1.0 / varianceB.get(0, 0);
    double weightBy = 1.0 / varianceB.get(1, 0);

    Pose2d fusedPose =
        new Pose2d(
            new Translation2d(
                (poseA.getTranslation().getX() * weightAx
                        + poseB.getTranslation().getX() * weightBx)
                    / (weightAx + weightBx),
                (poseA.getTranslation().getY() * weightAy
                        + poseB.getTranslation().getY() * weightBy)
                    / (weightAy + weightBy)),
            fusedHeading);

    Matrix<N3, N1> fusedStdDev =
        VecBuilder.fill(
            Math.sqrt(1.0 / (weightAx + weightBx)),
            Math.sqrt(1.0 / (weightAy + weightBy)),
            Math.sqrt(1.0 / (1.0 / varianceA.get(2, 0) + 1.0 / varianceB.get(2, 0))));

    return new VisionMeasurement(fusedPose, b.timestampSeconds(), fusedStdDev);
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
        mt2.avgTagArea,
        mt2.tagCount,
        mt2.timestampSeconds,
        tagDistRobotM,
        fpgaNow,
        speeds)) {
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
            VecBuilder.fill(std.sigmaXMeters(), std.sigmaYMeters(), std.sigmaYawRadians())));
  }

  private Optional<VisionMeasurement> processPhoton(double fpgaNow) {
    photonPoseEstimator.addHeadingData(fpgaNow, drive.getPose().getRotation());

    List<PhotonPipelineResult> unread = photonCamera.getAllUnreadResults();
    PhotonPipelineResult result =
        unread.stream()
            .filter(PhotonPipelineResult::hasTargets)
            .max(Comparator.comparingDouble(PhotonPipelineResult::getTimestampSeconds))
            .orElse(null);
    if (result == null) {
      logPhotonPoseEstimate(Optional.empty(), Optional.empty());
      return Optional.empty();
    }

    Optional<EstimatedRobotPose> multiOpt = photonPoseEstimator.estimateCoprocMultiTagPose(result);
    Optional<EstimatedRobotPose> pnpOpt =
        multiOpt.isPresent()
            ? Optional.empty()
            : photonPoseEstimator.estimatePnpDistanceTrigSolvePose(result);
    logPhotonPoseEstimate(multiOpt, pnpOpt);

    if (multiOpt.isPresent()) {
      EstimatedRobotPose est = multiOpt.get();
      Pose2d pose = est.estimatedPose.toPose2d();
      List<PhotonTrackedTarget> targets = est.targetsUsed;
      int tagCount = Math.max(1, targets.size());
      double yawRad = maxAbsPhotonYawRad(targets);
      double taMean = photonAvgAreaFraction(targets);


      double tagDistM = photonAvgCameraToTagDistanceMeters(targets);

      if (shouldReject(
          taMean,
          tagCount,
          est.timestampSeconds,
          tagDistM,
          fpgaNow,
          drive.getChassisSpeeds())) {
        return Optional.empty();
      }

      double xyStd = PoseEstimatorConstants.tAtoDev.get(taMean);
      return Optional.of(
          new VisionMeasurement(
              pose,
              est.timestampSeconds,
              VecBuilder.fill(xyStd, xyStd, VisionFusion.PHOTON_THETA_STDDEV_RADIANS)));
    }

    if (pnpOpt.isEmpty()) {
      return Optional.empty();
    }

    EstimatedRobotPose est = pnpOpt.get();
    Pose2d pose = est.estimatedPose.toPose2d();
    PhotonTrackedTarget best = result.getBestTarget();
    if (best == null) {
      return Optional.empty();
    }

    double taFrac = best.area / 100.0;
    double yawRad = Units.degreesToRadians(Math.abs(best.yaw));

    double tagDistM = photonCameraToTagDistanceMeters(best);

    if (shouldReject(
        taFrac,
        1,
        est.timestampSeconds,
        tagDistM,
        fpgaNow,
        drive.getChassisSpeeds())) {
      return Optional.empty();
    }

    double xyStd = PoseEstimatorConstants.tAtoDev.get(taFrac);
    return Optional.of(
        new VisionMeasurement(
            pose,
            est.timestampSeconds,
            VecBuilder.fill(xyStd, xyStd, VisionFusion.PHOTON_THETA_STDDEV_RADIANS)));
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
    return new LlStdDevs(
        xyStd, xyStd, VisionFusion.PHOTON_THETA_STDDEV_RADIANS, "TA_LOOKUP");
  }

  private static Optional<LlStdDevs> parseLimelightNtStdDevs(
      double[] arr, int xIndex, int yIndex, int yawIndex, String source) {
    if (arr.length <= yawIndex) {
      return Optional.empty();
    }
    double sigmaX = arr[xIndex];
    double sigmaY = arr[yIndex];
    double sigmaYawDeg = arr[yawIndex];
    if (!Double.isFinite(sigmaX)
        || !Double.isFinite(sigmaY)
        || !Double.isFinite(sigmaYawDeg)) {
      return Optional.empty();
    }
    if (sigmaX <= 0.0 || sigmaY <= 0.0 || sigmaYawDeg <= 0.0) {
      return Optional.empty();
    }
    return Optional.of(
        new LlStdDevs(sigmaX, sigmaY, Units.degreesToRadians(sigmaYawDeg), source));
  }

  private record VisionMeasurement(
      Pose2d pose, double timestampSeconds, Matrix<N3, N1> stdDevs) {}

  private record LlStdDevs(
      double sigmaXMeters, double sigmaYMeters, double sigmaYawRadians, String source) {}

  /**
   * Logs Photon estimated pose (multi-tag preferred, else PNP distance trig) for AdvantageScope.
   */
  private static void logPhotonPoseEstimate(
      Optional<EstimatedRobotPose> multiOpt, Optional<EstimatedRobotPose> pnpOpt) {
    String base = "VisionFusion/PoseEstimates/Photon";
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
   * <p>{@code tagDistanceMeters}: Limelight uses mean {@link LimelightHelpers.RawFiducial#distToRobot}
   * when fiducials exist; otherwise {@link LimelightHelpers.PoseEstimate#avgTagDist}. Photon uses mean
   * camera–tag range from {@code getBestCameraToTarget()} (meters).
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
   * Tag–robot distance from Limelight: arithmetic mean {@link LimelightHelpers.RawFiducial#distToRobot};
   * if fiducials are missing uses {@link LimelightHelpers.PoseEstimate#avgTagDist} (meters).
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
