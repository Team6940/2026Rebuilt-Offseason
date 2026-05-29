package frc.robot.subsystems.Chassis;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

/** Per-cycle trench geometry for shared-control translation assist (HybridTrench only). */
public record HybridTrenchReference(
    TrenchLane lane,
    Translation2d guidanceVector,
    Translation2d lookaheadPoint,
    Translation2d nearestPoint,
    /** +1 = lookahead with path parameterization; -1 = reverse along path. */
    double pathTraversalSign,
    Pose2d robotPose) {}
