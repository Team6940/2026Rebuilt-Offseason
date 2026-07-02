package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import org.littletonrobotics.junction.Logger;

/**
 * Pre-publishes AdvantageKit keys that are first written from a hot control path (e.g.
 * HybridTrench). On the roboRIO with {@link
 * org.littletonrobotics.junction.networktables.NT4Publisher}, the first {@link Logger#recordOutput}
 * for each key creates an NT4 publisher synchronously; doing that during teleop can stall the robot
 * loop for seconds.
 */
public final class NtTelemetryBootstrap {
  private static final int kModuleCount = 4;

  private NtTelemetryBootstrap() {}

  private static SwerveModuleState[] zeroSwerveModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[kModuleCount];
    for (int i = 0; i < kModuleCount; i++) {
      states[i] = new SwerveModuleState();
    }
    return states;
  }

  private static SwerveModulePosition[] zeroSwerveModulePositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[kModuleCount];
    for (int i = 0; i < kModuleCount; i++) {
      positions[i] = new SwerveModulePosition();
    }
    return positions;
  }

  /** Call once after {@link Logger#start()} while NT publishing is enabled. */
  public static void preannounceHotPathTopics() {
    Logger.recordOutput("HybridTrench/PathsFlipped", false);
    Logger.recordOutput("HybridTrench/Lane", "");
    Logger.recordOutput("HybridTrench/PathTraversalSign", 0.0);
    Logger.recordOutput("HybridTrench/GuidanceVector", new Translation2d());
    Logger.recordOutput("HybridTrench/Lookahead", Pose2d.kZero);
    Logger.recordOutput("HybridTrench/Nearest", Pose2d.kZero);
    Logger.recordOutput("HybridTrench/DesiredChassisHeading", Rotation2d.kZero);
    Logger.recordOutput("Drive/DesiredFacing", Rotation2d.kZero);
    Logger.recordOutput("Drive/BlendedFieldLinear", new Translation2d());
    Logger.recordOutput("Odometry/Robot", Pose2d.kZero);
    Logger.recordOutput("SwerveChassisSpeeds/Measured", new ChassisSpeeds());
    Logger.recordOutput("SwerveChassisSpeeds/Setpoints", new ChassisSpeeds());
    Logger.recordOutput("SwerveStates/Measured", zeroSwerveModuleStates());
    Logger.recordOutput("SwerveStates/SetpointsOptimized", zeroSwerveModuleStates());
    Logger.recordOutput("DriveState/ModulePositions", zeroSwerveModulePositions());
    Logger.recordOutput("DriveState/Timestamp", 0.0);
    Logger.recordOutput("DriveState/OdometryPeriod", 0.0);
    Logger.recordOutput("DriveState/OdometryFrequency", 0.0);
    Logger.recordOutput("DriveState/FailedDaqs", 0);
  }
}
