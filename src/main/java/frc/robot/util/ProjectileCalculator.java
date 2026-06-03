package frc.robot.util;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants.ProjectileConstants;

/**
 * SCORE uses static distance tables only. PASS uses the iterative motion solver ({@link #solve}).
 */
public final class ProjectileCalculator {
  private static final int LOOKAHEAD_ITERATIONS = 20;

  private ProjectileCalculator() {}

  /** Static SCORE lookup: hood angle (deg) from distance (m). */
  public static double getHoodTargetDegs(double distanceMeters) {
    return ProjectileConstants.DistanceToHoodDegs.get(distanceMeters);
  }

  /** Static SCORE lookup: shooter velocity (RPS) from distance (m). */
  public static double getShooterTargetVelocity(double distanceMeters) {
    return ProjectileConstants.DistanceToShooterRps.get(distanceMeters);
  }

  public record ShotSolution(
      Translation2d virtualTarget,
      Rotation2d aimAngle,
      double lookaheadDistance,
      double shooterRps,
      double hoodAngleDeg,
      double flightTimeSecs) {}

  /**
   * PASS motion solver: finds virtual target so the note meets the real target after robot motion.
   */
  public static ShotSolution solve(
      Translation2d shooterPosition, Translation2d realTarget, Translation2d fieldVelocity) {
    double lookaheadDistance = shooterPosition.getDistance(realTarget);

    for (int i = 0; i < LOOKAHEAD_ITERATIONS; i++) {
      double tof = getPassFlightTime(lookaheadDistance);
      double lx = shooterPosition.getX() + fieldVelocity.getX() * tof;
      double ly = shooterPosition.getY() + fieldVelocity.getY() * tof;
      lookaheadDistance = Math.hypot(lx - realTarget.getX(), ly - realTarget.getY());
    }

    double tof = getPassFlightTime(lookaheadDistance);
    Translation2d virtualTarget =
        new Translation2d(
            realTarget.getX() - fieldVelocity.getX() * tof,
            realTarget.getY() - fieldVelocity.getY() * tof);
    Rotation2d aimAngle = virtualTarget.minus(shooterPosition).getAngle();
    double rps = getPassShooterRps(lookaheadDistance);
    double hoodDeg = getPassHoodDegs(lookaheadDistance);

    return new ShotSolution(virtualTarget, aimAngle, lookaheadDistance, rps, hoodDeg, tof);
  }

  private static double getPassFlightTime(double distanceMeters) {
    return ProjectileConstants.DistanceToFlightTimeSecs.get(distanceMeters);
  }

  private static double getPassShooterRps(double distanceMeters) {
    return ProjectileConstants.PassDistanceToShooterRps.get(distanceMeters);
  }

  private static double getPassHoodDegs(double distanceMeters) {
    return ProjectileConstants.PassDistanceToHoodDegs.get(distanceMeters);
  }
}
