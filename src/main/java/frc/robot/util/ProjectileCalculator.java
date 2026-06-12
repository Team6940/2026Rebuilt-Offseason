package frc.robot.util;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.ProjectileConstants;

/**
 * Ballistic lookups and shot planning for hybrid shoot ({@link
 * frc.robot.commands.HybridShootCommand}).
 *
 * <p>SCORE and PASS both use the iterative motion solver ({@link #solve}) with profile-specific
 * distance tables.
 */
public final class ProjectileCalculator {
  private static final int LOOKAHEAD_ITERATIONS = 20;

  private enum ShotProfile {
    SCORE,
    PASS
  }

  private ProjectileCalculator() {}

  /** Immutable ballistic setpoints; operator trims are applied by the command each cycle. */
  public static final class ShotPlan {
    public final boolean usesMotionSolver;
    public final Translation2d target;
    public final double distanceMeters;
    public final Rotation2d heading;
    public final double hoodDegs;
    public final double shooterRps;
    public final Translation2d virtualTarget;

    private ShotPlan(
        boolean usesMotionSolver,
        Translation2d target,
        double distanceMeters,
        Rotation2d heading,
        double hoodDegs,
        double shooterRps,
        Translation2d virtualTarget) {
      this.usesMotionSolver = usesMotionSolver;
      this.target = target;
      this.distanceMeters = distanceMeters;
      this.heading = heading;
      this.hoodDegs = hoodDegs;
      this.shooterRps = shooterRps;
      this.virtualTarget = virtualTarget;
    }
  }

  /** Static SCORE lookup: hood angle (deg) from distance (m). */
  public static double getHoodTargetDegs(double distanceMeters) {
    return ProjectileConstants.DistanceToHoodDegs.get(distanceMeters);
  }

  /** Static SCORE lookup: shooter velocity (RPS) from distance (m). */
  public static double getShooterTargetVelocity(double distanceMeters) {
    return ProjectileConstants.DistanceToShooterRps.get(distanceMeters);
  }

  /** Robot chassis heading so a rear-facing shooter aims at {@code towardTarget}. */
  private static Rotation2d chassisHeadingForShooter(Rotation2d towardTarget) {
    return towardTarget.plus(Rotation2d.kPi);
  }

  /** Hub shot: motion solver compensates for chassis velocity while aiming at alliance hub center. */
  public static ShotPlan planScore(
      Translation2d shooterPosition, Translation2d hubCenter, Translation2d fieldVelocity) {
    ShotSolution sol = solve(shooterPosition, hubCenter, fieldVelocity, ShotProfile.SCORE);
    return new ShotPlan(
        true,
        hubCenter,
        sol.lookaheadDistance(),
        sol.aimAngle(),
        sol.hoodAngleDeg(),
        sol.shooterRps(),
        sol.virtualTarget());
  }

  /**
   * Pass shot: lob beside the hub into the open bump lane. Target Y follows robot side; motion
   * solver compensates for chassis velocity.
   */
  public static ShotPlan planPass(Translation2d shooterPosition, Translation2d fieldVelocity) {
    Translation2d passTarget = resolvePassTarget(shooterPosition);
    ShotSolution sol = solve(shooterPosition, passTarget, fieldVelocity, ShotProfile.PASS);
    return new ShotPlan(
        true,
        passTarget,
        sol.lookaheadDistance(),
        sol.aimAngle(),
        sol.hoodAngleDeg(),
        sol.shooterRps(),
        sol.virtualTarget());
  }

  /** Pass lane target beside the alliance hub (same geometry as 2026 game-dev hybrid pass). */
  public static Translation2d resolvePassTarget(Translation2d shooterPosition) {
    boolean isBlue =
        DriverStation.getAlliance().isPresent()
            && DriverStation.getAlliance().get() == Alliance.Blue;
    double passX = isBlue ? 0.5 : FieldConstants.fieldLength - 0.5;
    double leftBumpCenterY =
        (FieldConstants.LinesHorizontal.leftBumpStart + FieldConstants.LinesHorizontal.leftBumpEnd)
                / 2.0
            + 1.0;
    double rightBumpCenterY =
        (FieldConstants.LinesHorizontal.rightBumpStart
                    + FieldConstants.LinesHorizontal.rightBumpEnd)
                / 2.0
            - 1.0;
    double fieldCenterY = FieldConstants.fieldWidth / 2.0;
    return shooterPosition.getY() > fieldCenterY
        ? new Translation2d(passX, leftBumpCenterY)
        : new Translation2d(passX, rightBumpCenterY);
  }

  public record ShotSolution(
      Translation2d virtualTarget,
      Rotation2d aimAngle,
      double lookaheadDistance,
      double shooterRps,
      double hoodAngleDeg,
      double flightTimeSecs) {}

  /**
   * Motion solver: finds virtual target so the note meets the real target after robot motion.
   */
  public static ShotSolution solve(
      Translation2d shooterPosition,
      Translation2d realTarget,
      Translation2d fieldVelocity,
      ShotProfile profile) {
    double lookaheadDistance = shooterPosition.getDistance(realTarget);

    for (int i = 0; i < LOOKAHEAD_ITERATIONS; i++) {
      double tof = getFlightTime(profile, lookaheadDistance);
      double lx = shooterPosition.getX() + fieldVelocity.getX() * tof;
      double ly = shooterPosition.getY() + fieldVelocity.getY() * tof;
      lookaheadDistance = Math.hypot(lx - realTarget.getX(), ly - realTarget.getY());
    }

    double tof = getFlightTime(profile, lookaheadDistance);
    Translation2d virtualTarget =
        new Translation2d(
            realTarget.getX() - fieldVelocity.getX() * tof,
            realTarget.getY() - fieldVelocity.getY() * tof);
    Rotation2d aimAngle = chassisHeadingForShooter(virtualTarget.minus(shooterPosition).getAngle());
    double rps = getShooterRps(profile, lookaheadDistance);
    double hoodDeg = getHoodDegs(profile, lookaheadDistance);

    return new ShotSolution(virtualTarget, aimAngle, lookaheadDistance, rps, hoodDeg, tof);
  }

  private static double getFlightTime(ShotProfile profile, double distanceMeters) {
    return ProjectileConstants.DistanceToFlightTimeSecs.get(distanceMeters);
  }

  private static double getShooterRps(ShotProfile profile, double distanceMeters) {
    return profile == ShotProfile.SCORE
        ? getShooterTargetVelocity(distanceMeters)
        : ProjectileConstants.PassDistanceToShooterRps.get(distanceMeters);
  }

  private static double getHoodDegs(ShotProfile profile, double distanceMeters) {
    return profile == ShotProfile.SCORE
        ? getHoodTargetDegs(distanceMeters)
        : ProjectileConstants.PassDistanceToHoodDegs.get(distanceMeters);
  }
}
