package frc.robot.subsystems.Vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Constants;
import frc.robot.Constants.PoseEstimatorConstants;
import frc.robot.Constants.VisionFusion;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Chassis.CommandSwerveDrivetrain;
import java.util.Comparator;
import java.util.List;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

/** Reads Limelight and PhotonVision hardware into {@link VisionIO.VisionCameraInputs}. */
public class VisionIOLive implements VisionIO {

  private static final double[] LL_DEFAULT_STDDEVS =
      new double[VisionFusion.LL_STDDEV_ARRAY_LENGTH];

  private final PhotonCamera photonFront;

  private final NetworkTableEntry llLeftSnapshot =
      NetworkTableInstance.getDefault().getTable(RobotContainer.limelightBack).getEntry("snapshot");

  // private final NetworkTableEntry llRightSnapshot =
  //     NetworkTableInstance.getDefault()
  //         .getTable(RobotContainer.limelightRight)
  //         .getEntry("snapshot");

  public VisionIOLive() {
    photonFront = new PhotonCamera(RobotContainer.photonCameraFront);
    configurePhotonNetworkTables(photonFront);
  }

  /** PhotonVision subtable keys under {@code photonvision/<cameraName>/}. */
  private static void configurePhotonNetworkTables(PhotonCamera driverCamera) {
    driverCamera.setDriverMode(true);

    NetworkTableInstance nt = NetworkTableInstance.getDefault();
    nt.getTable("photonvision")
        .getSubTable(RobotContainer.photonCameraFront)
        .getEntry("driverMode")
        .setBoolean(true);
  }

  @Override
  public void updateInputs(
      VisionCameraInputs limelightBack,
      VisionCameraInputs photonFrontInputs,
      CommandSwerveDrivetrain drive,
      double fpgaNow) {
    llLeftSnapshot.setNumber(0);
    // llRightSnapshot.setNumber(0);

    updateLimelight(
        limelightBack, RobotContainer.limelightBack, "Left", llLeftSnapshot, drive, fpgaNow);
    // updateLimelight(
    //     limelightRight, RobotContainer.limelightRight, "Right", llRightSnapshot, drive, fpgaNow);
    updatePhotonDriver(photonFrontInputs, photonFront);
  }

  private static void clearInputs(VisionCameraInputs inputs) {
    inputs.connected = false;
    inputs.seesTarget = false;
    inputs.hasMeasurement = false;
    inputs.pose = new Pose2d();
    inputs.timestampSeconds = 0.0;
    inputs.stdDevX = 0.0;
    inputs.stdDevY = 0.0;
    inputs.stdDevYawRad = 0.0;
    inputs.tagCount = 0;
    inputs.tagIdCount = 0;
    inputs.tagId0 = -1;
    inputs.tagId1 = -1;
    inputs.tagId2 = -1;
    inputs.tagId3 = -1;
    inputs.measurementSource = 0;
    inputs.sourceLabel = "";
    inputs.stdDevSource = "";
  }

  private void updateLimelight(
      VisionCameraInputs inputs,
      String limelightName,
      String logSide,
      NetworkTableEntry snapshotEntry,
      CommandSwerveDrivetrain drive,
      double fpgaNow) {
    clearInputs(inputs);
    drive.applyLimelightGyroForMegaTag2(limelightName);

    LimelightHelpers.PoseEstimate mt2 =
        LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName);
    logLimelightPoseEstimate(logSide, mt2);

    if (mt2 == null) {
      Logger.recordOutput("VisionFusion/Limelight/" + limelightName + "/Connected", false);
      return;
    }

    inputs.connected = true;
    Logger.recordOutput("VisionFusion/Limelight/" + limelightName + "/Connected", true);

    if (!LimelightHelpers.getTV(limelightName)) {
      Logger.recordOutput("VisionFusion/Limelight/" + limelightName + "/SeesTarget", false);
      Logger.recordOutput("VisionFusion/Limelight/" + limelightName + "/Accepted", false);
      return;
    }
    inputs.seesTarget = true;
    Logger.recordOutput("VisionFusion/Limelight/" + limelightName + "/SeesTarget", true);

    if (mt2.tagCount <= 0) {
      Logger.recordOutput("VisionFusion/Limelight/" + limelightName + "/Accepted", false);
      return;
    }

    snapshotEntry.setNumber(1);
    ChassisSpeeds speeds = drive.getChassisSpeeds();
    double tagDistRobotM = VisionSubsystem.limelightTagDistanceToRobotMeters(mt2);

    if (VisionSubsystem.shouldReject(
        mt2.avgTagArea, mt2.tagCount, mt2.timestampSeconds, tagDistRobotM, fpgaNow, speeds)) {
      Logger.recordOutput("VisionFusion/Limelight/" + limelightName + "/Accepted", false);
      return;
    }

    VisionSubsystem.LlStdDevs std = resolveLimelightStdDevs(limelightName, mt2.avgTagArea);
    Logger.recordOutput("VisionFusion/Limelight/" + limelightName + "/StdDevsValid", true);
    Logger.recordOutput("VisionFusion/Limelight/" + limelightName + "/StdDevX", std.sigmaXMeters());
    Logger.recordOutput("VisionFusion/Limelight/" + limelightName + "/StdDevY", std.sigmaYMeters());
    Logger.recordOutput(
        "VisionFusion/Limelight/" + limelightName + "/StdDevYawRad", std.sigmaYawRadians());
    Logger.recordOutput("VisionFusion/Limelight/" + limelightName + "/StdDevSource", std.source());
    Logger.recordOutput("VisionFusion/Limelight/" + limelightName + "/Accepted", true);

    fillMeasurement(
        inputs,
        mt2.pose,
        mt2.timestampSeconds,
        std.sigmaXMeters(),
        std.sigmaYMeters(),
        std.sigmaYawRadians(),
        VisionSubsystem.MeasurementSource.LIMELIGHT_MEGATAG2,
        "Limelight" + logSide,
        std.source(),
        mt2.tagCount,
        VisionSubsystem.limelightTagIds(mt2));
  }

  private static void updatePhotonDriver(VisionCameraInputs inputs, PhotonCamera driverCamera) {
    clearInputs(inputs);
    String cameraName = driverCamera.getName();
    inputs.connected = driverCamera.isConnected();
    Logger.recordOutput("VisionFusion/Photon/" + cameraName + "/Connected", inputs.connected);
    Logger.recordOutput("VisionFusion/Photon/" + cameraName + "/DriverMode", true);
  }

  // private void updatePhoton(
  //     VisionCameraInputs inputs,
  //     PhotonCameraSetup setup,
  //     double fpgaNow,
  //     CommandSwerveDrivetrain drive) {
  //   clearInputs(inputs);
  //   String cameraName = setup.camera.getName();
  //   setup.poseEstimator.addHeadingData(fpgaNow, drive.getPose().getRotation());

  //   List<PhotonPipelineResult> unread = setup.camera.getAllUnreadResults();
  //   PhotonPipelineResult result =
  //       unread.stream()
  //           .filter(PhotonPipelineResult::hasTargets)
  //           .max(Comparator.comparingDouble(PhotonPipelineResult::getTimestampSeconds))
  //           .orElse(null);
  //   if (result == null) {
  //     logPhotonPoseEstimate(setup.logSide, Optional.empty(), Optional.empty());
  //     Logger.recordOutput(
  //         "VisionFusion/Photon/" + cameraName + "/Connected", setup.camera.isConnected());
  //     Logger.recordOutput("VisionFusion/Photon/" + cameraName + "/SeesTarget", false);
  //     Logger.recordOutput("VisionFusion/Photon/" + cameraName + "/Accepted", false);
  //     return;
  //   }

  //   inputs.connected = setup.camera.isConnected();
  //   inputs.seesTarget = true;
  //   Logger.recordOutput(
  //       "VisionFusion/Photon/" + cameraName + "/Connected", setup.camera.isConnected());
  //   Logger.recordOutput("VisionFusion/Photon/" + cameraName + "/SeesTarget", true);

  //   Optional<EstimatedRobotPose> multiOpt =
  // setup.poseEstimator.estimateCoprocMultiTagPose(result);
  //   Optional<EstimatedRobotPose> pnpOpt =
  //       multiOpt.isPresent()
  //           ? Optional.empty()
  //           : setup.poseEstimator.estimatePnpDistanceTrigSolvePose(result);
  //   logPhotonPoseEstimate(setup.logSide, multiOpt, pnpOpt);

  //   if (multiOpt.isPresent()) {
  //     EstimatedRobotPose est = multiOpt.get();
  //     List<PhotonTrackedTarget> targets = est.targetsUsed;
  //     int tagCount = Math.max(1, targets.size());
  //     double taMean = VisionSubsystem.photonAvgAreaFraction(targets);
  //     double tagDistM = VisionSubsystem.photonAvgCameraToTagDistanceMeters(targets);

  //     if (VisionSubsystem.shouldReject(
  //         taMean, tagCount, est.timestampSeconds, tagDistM, fpgaNow, drive.getChassisSpeeds())) {
  //       Logger.recordOutput("VisionFusion/Photon/" + cameraName + "/Accepted", false);
  //       return;
  //     }

  //     double xyStd = PoseEstimatorConstants.tAtoDev.get(taMean);
  //     Logger.recordOutput("VisionFusion/Photon/" + cameraName + "/Accepted", true);
  //     fillMeasurement(
  //         inputs,
  //         est.estimatedPose.toPose2d(),
  //         est.timestampSeconds,
  //         xyStd,
  //         xyStd,
  //         VisionFusion.PHOTON_THETA_STDDEV_RADIANS,
  //         VisionSubsystem.MeasurementSource.PHOTON,
  //         "Photon" + setup.logSide,
  //         "TA_LOOKUP",
  //         tagCount,
  //         VisionSubsystem.photonTagIds(targets));
  //     return;
  //   }

  //   if (pnpOpt.isEmpty()) {
  //     Logger.recordOutput("VisionFusion/Photon/" + cameraName + "/Accepted", false);
  //     return;
  //   }

  //   EstimatedRobotPose est = pnpOpt.get();
  //   PhotonTrackedTarget best = result.getBestTarget();
  //   if (best == null) {
  //     Logger.recordOutput("VisionFusion/Photon/" + cameraName + "/Accepted", false);
  //     return;
  //   }

  //   double taFrac = best.area / 100.0;
  //   double tagDistM = VisionSubsystem.photonCameraToTagDistanceMeters(best);

  //   if (VisionSubsystem.shouldReject(
  //       taFrac, 1, est.timestampSeconds, tagDistM, fpgaNow, drive.getChassisSpeeds())) {
  //     Logger.recordOutput("VisionFusion/Photon/" + cameraName + "/Accepted", false);
  //     return;
  //   }

  //   double xyStd = PoseEstimatorConstants.tAtoDev.get(taFrac);
  //   Logger.recordOutput("VisionFusion/Photon/" + cameraName + "/Accepted", true);
  //   fillMeasurement(
  //       inputs,
  //       est.estimatedPose.toPose2d(),
  //       est.timestampSeconds,
  //       xyStd,
  //       xyStd,
  //       VisionFusion.PHOTON_THETA_STDDEV_RADIANS,
  //       VisionSubsystem.MeasurementSource.PHOTON,
  //       "Photon" + setup.logSide,
  //       "TA_LOOKUP",
  //       1,
  //       new int[] {best.getFiducialId()});
  // }

  private static void fillMeasurement(
      VisionCameraInputs inputs,
      Pose2d pose,
      double timestampSeconds,
      double stdDevX,
      double stdDevY,
      double stdDevYawRad,
      VisionSubsystem.MeasurementSource source,
      String sourceLabel,
      String stdDevSource,
      int tagCount,
      int[] tagIds) {
    inputs.hasMeasurement = true;
    inputs.pose = pose;
    inputs.timestampSeconds = timestampSeconds;
    inputs.stdDevX = stdDevX;
    inputs.stdDevY = stdDevY;
    inputs.stdDevYawRad = stdDevYawRad;
    inputs.measurementSource = source.ordinal();
    inputs.sourceLabel = sourceLabel;
    inputs.stdDevSource = stdDevSource;
    inputs.tagCount = tagCount;
    inputs.tagIdCount = Math.min(tagIds.length, 4);
    if (tagIds.length > 0) {
      inputs.tagId0 = tagIds[0];
    }
    if (tagIds.length > 1) {
      inputs.tagId1 = tagIds[1];
    }
    if (tagIds.length > 2) {
      inputs.tagId2 = tagIds[2];
    }
    if (tagIds.length > 3) {
      inputs.tagId3 = tagIds[3];
    }
  }

  private static VisionSubsystem.LlStdDevs resolveLimelightStdDevs(
      String limelightName, double avgTagArea) {
    double[] arr =
        LimelightHelpers.getLimelightNTTableEntry(limelightName, "stddevs")
            .getDoubleArray(LL_DEFAULT_STDDEVS);

    Optional<VisionSubsystem.LlStdDevs> mt2 =
        VisionSubsystem.parseLimelightNtStdDevs(
            arr,
            VisionFusion.LL_MT2_X_STDDEV_INDEX,
            VisionFusion.LL_MT2_Y_STDDEV_INDEX,
            VisionFusion.LL_MT2_YAW_STDDEV_INDEX,
            "MT2");
    if (mt2.isPresent()) {
      return mt2.get();
    }

    // Optional<VisionSubsystem.LlStdDevs> mt1 =
    //     VisionSubsystem.parseLimelightNtStdDevs(
    //         arr,
    //         VisionFusion.LL_MT1_X_STDDEV_INDEX,
    //         VisionFusion.LL_MT1_Y_STDDEV_INDEX,
    //         VisionFusion.LL_MT1_YAW_STDDEV_INDEX,
    //         "MT1");
    // if (mt1.isPresent()) {
    //   return mt1.get();
    // }

    double taFrac = avgTagArea / 100.0;
    double xyStd = PoseEstimatorConstants.tAtoDev.get(taFrac);
    return new VisionSubsystem.LlStdDevs(
        xyStd, xyStd, VisionFusion.PHOTON_THETA_STDDEV_RADIANS, "TA_LOOKUP");
  }

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
}
