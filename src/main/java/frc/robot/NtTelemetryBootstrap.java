package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import org.littletonrobotics.junction.Logger;

/**
 * Pre-publishes AdvantageKit keys that are first written from a hot control path (e.g.
 * HybridTrench). On the roboRIO with {@link
 * org.littletonrobotics.junction.networktables.NT4Publisher}, the first {@link Logger#recordOutput}
 * for each key creates an NT4 publisher synchronously; doing that during teleop can stall the robot
 * loop for seconds.
 */
public final class NtTelemetryBootstrap {
  private NtTelemetryBootstrap() {}

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
  }
}
