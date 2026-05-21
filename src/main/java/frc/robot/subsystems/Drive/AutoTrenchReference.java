package frc.robot.subsystems.Drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/** Geometric trench guidance for one control cycle (no trajectory timing). */
public record AutoTrenchReference(
    TrenchLane lane,
    Translation2d guidanceVector,
    Translation2d lookaheadPoint,
    Translation2d nearestPoint,
    double distanceAlongPath,
    /** +1 = lookahead ahead along path parameterization, -1 = behind (reverse entry). */
    double pathTraversalSign,
    Pose2d robotPose) {

  public Rotation2d guidanceDirection() {
    if (guidanceVector.getNorm() < 1e-6) {
      return new Rotation2d();
    }
    return new Rotation2d(
        Math.atan2(guidanceVector.getY(), guidanceVector.getX()));
  }
}
