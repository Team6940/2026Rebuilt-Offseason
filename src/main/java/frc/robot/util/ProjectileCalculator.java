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
 * <p>SCORE uses static distance tables. PASS uses the iterative motion solver ({@link #solve}).
 */
public final class ProjectileCalculator {
  private static final int LOOKAHEAD_ITERATIONS = 20;

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

  /** Hub shot: distance tables only, aim straight at alliance hub center. */
  public static ShotPlan planScore(Translation2d shooterPosition, Translation2d hubCenter) {
    double distanceMeters = shooterPosition.getDistance(hubCenter);
    return new ShotPlan(
        false,
        hubCenter,
        distanceMeters,
        chassisHeadingForShooter(hubCenter.minus(shooterPosition).getAngle()),
        getHoodTargetDegs(distanceMeters),
        getShooterTargetVelocity(distanceMeters),
        null);
  }

  /**
   * Pass shot: lob beside the hub into the open bump lane. Target Y follows robot side; motion
   * solver compensates for chassis velocity.
   */
  public static ShotPlan planPass(Translation2d shooterPosition, Translation2d fieldVelocity) {
    Translation2d passTarget = resolvePassTarget(shooterPosition);
    ShotSolution sol = solve(shooterPosition, passTarget, fieldVelocity);
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
    Rotation2d aimAngle = chassisHeadingForShooter(virtualTarget.minus(shooterPosition).getAngle());
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
