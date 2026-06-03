package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.Chassis.CommandSwerveDrivetrain;
import frc.robot.subsystems.Hood.HoodSubsystem;
import frc.robot.subsystems.ImprovedCommandXboxController;
import frc.robot.subsystems.Indexer.IndexerSubsystem;
import frc.robot.subsystems.Shooter.ShooterSubsystem;
import frc.robot.subsystems.SuperStructure;
import frc.robot.subsystems.SuperStructure.ControlMode;
import frc.robot.subsystems.SuperStructure.IntakeMode;
import frc.robot.subsystems.SuperStructure.ShootPhase;
import frc.robot.util.ProjectileCalculator;
import frc.robot.util.ProjectileCalculator.ShotSolution;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

final class HybridShootCommandConstants {
  private HybridShootCommandConstants() {}

  static final double HeadingToleranceDegs = 3.0;
  static final double HoodToleranceDegs = 3.0;
  static final double ShooterToleranceRps = 2.5;

  static final double AimHeadingCompRangeDegs = 10.0;
  static final double HoodCompRangeDegs = 3.0;

  static final double ShootHeadingFineTuneDeadband = 0.3;
  static final double FeedDurationSec = 2.0;
  static final double PostRetractWaitSec = 1.0;
}

/**
 * Hybrid shoot state machine ({@link ShootPhase}):
 *
 * <pre>
 * OFF ──(RB/Y)──► AIM ──(at target)──► READY ──(RT)──► SHOOT
 *                    ▲                    │              │
 *                    └──(lost target)─────┘              │
 *                    ▲◄──────(RT release)────────────────┘
 * OFF ◄──(RB/Y release)── AIM/READY/SHOOT
 * </pre>
 *
 * HEATUP is handled separately by {@link HeatupCommand}.
 */
public class HybridShootCommand extends Command {
  /** Feed/retract sub-steps while {@link ShootPhase#SHOOT}. */
  private enum ShootSequence {
    FEEDING,
    RETRACT_WAIT,
    COMPLETE
  }

  private final CommandSwerveDrivetrain drive;
  private final BooleanSupplier aimScoreSupplier;
  private final BooleanSupplier aimPassSupplier;
  private final BooleanSupplier fireSupplier;
  private final DoubleSupplier driverXSupplier;
  private final DoubleSupplier driverYSupplier;
  private final DoubleSupplier operatorHoodAxisSupplier;
  private final DoubleSupplier operatorAimHeadingAxisSupplier;
  private final BooleanSupplier operatorBPressed;
  private final BooleanSupplier operatorAPressed;
  private final BooleanSupplier operatorXPressed;
  private final BooleanSupplier operatorYPressed;
  private final HoodSubsystem hood = HoodSubsystem.getInstance();
  private final ShooterSubsystem shooter = ShooterSubsystem.getInstance();
  private final IndexerSubsystem indexer = IndexerSubsystem.getInstance();
  private final SuperStructure superStructure = SuperStructure.getInstance();

  private ShootSequence shootSequence = ShootSequence.FEEDING;
  private double shootSequenceStartSec = 0.0;
  private double rpsOffset = 0.0;

  public HybridShootCommand(
      CommandSwerveDrivetrain drive,
      BooleanSupplier aimScoreSupplier,
      BooleanSupplier aimPassSupplier,
      BooleanSupplier fireSupplier,
      DoubleSupplier driverXSupplier,
      DoubleSupplier driverYSupplier,
      DoubleSupplier operatorHoodAxisSupplier,
      DoubleSupplier operatorAimHeadingAxisSupplier,
      BooleanSupplier operatorBPressed,
      BooleanSupplier operatorAPressed,
      BooleanSupplier operatorXPressed,
      BooleanSupplier operatorYPressed) {
    this.drive = drive;
    this.aimScoreSupplier = aimScoreSupplier;
    this.aimPassSupplier = aimPassSupplier;
    this.fireSupplier = fireSupplier;
    this.driverXSupplier = driverXSupplier;
    this.driverYSupplier = driverYSupplier;
    this.operatorHoodAxisSupplier = operatorHoodAxisSupplier;
    this.operatorAimHeadingAxisSupplier = operatorAimHeadingAxisSupplier;
    this.operatorBPressed = operatorBPressed;
    this.operatorAPressed = operatorAPressed;
    this.operatorXPressed = operatorXPressed;
    this.operatorYPressed = operatorYPressed;
    addRequirements(drive, hood, shooter, indexer);
  }

  @Override
  public void initialize() {
    shootSequence = ShootSequence.FEEDING;
    shootSequenceStartSec = 0.0;
    rpsOffset = 0.0;
    hood.setOperatorInputScalar(0.0);
    transitionTo(ShootPhase.AIM);
  }

  @Override
  public void execute() {
    updateControlMode();

    ShotPlan plan = ShotPlanner.plan(superStructure.getControlMode(), drive);
    applyOperatorOffsets(plan);

    boolean ready = isAtTargetAngle(plan) && isAtTargetHoodDegs(plan) && isAtTargetShootVelocity(plan);
    boolean fire = fireSupplier.getAsBoolean();

    switch (superStructure.getShootPhase()) {
      case AIM -> runAimState(plan, ready);
      case READY -> runReadyState(plan, ready, fire);
      case SHOOT -> runShootState(plan, ready, fire);
      default -> transitionTo(ShootPhase.AIM);
    }

    logShot(plan, ready, fire);
  }

  @Override
  public void end(boolean interrupted) {
    cleanup();
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  private void updateControlMode() {
    if (aimScoreSupplier.getAsBoolean()) {
      superStructure.setControlMode(ControlMode.SCORE);
    } else if (aimPassSupplier.getAsBoolean()) {
      superStructure.setControlMode(ControlMode.PASS);
    }
  }

  private void runAimState(ShotPlan plan, boolean ready) {
    applyMechanisms(plan, false);
    if (ready) {
      transitionTo(ShootPhase.READY);
    }
  }

  private void runReadyState(ShotPlan plan, boolean ready, boolean fire) {
    applyMechanisms(plan, false);
    if (!ready) {
      transitionTo(ShootPhase.AIM);
    } else if (fire) {
      transitionTo(ShootPhase.SHOOT);
    }
  }

  private void runShootState(ShotPlan plan, boolean ready, boolean fire) {
    plan.shootHeadingFineTuneDegs =
        ImprovedCommandXboxController.applyInputCurve(operatorAimHeadingAxisSupplier.getAsDouble());
    applyMechanisms(plan, true);
    advanceShootSequence();

    if (!fire) {
      abortShootSequence();
      transitionTo(ready ? ShootPhase.READY : ShootPhase.AIM);
    }
  }

  private void transitionTo(ShootPhase next) {
    ShootPhase current = superStructure.getShootPhase();
    if (current == next) {
      return;
    }
    if (current == ShootPhase.SHOOT) {
      abortShootSequence();
    }
    if (next == ShootPhase.SHOOT) {
      beginShootSequence();
    }
    superStructure.setShootPhase(next);
    Logger.recordOutput("Cmds/HybridShoot/StateTransition", current + "->" + next);
  }

  private void beginShootSequence() {
    shootSequence = ShootSequence.FEEDING;
    shootSequenceStartSec = Timer.getFPGATimestamp();
    indexer.feed();
  }

  private void advanceShootSequence() {
    double now = Timer.getFPGATimestamp();
    switch (shootSequence) {
      case FEEDING -> {
        if (now - shootSequenceStartSec >= HybridShootCommandConstants.FeedDurationSec) {
          superStructure.setIntakeMode(IntakeMode.RETRACTED);
          shootSequence = ShootSequence.RETRACT_WAIT;
          shootSequenceStartSec = now;
        }
      }
      case RETRACT_WAIT -> {
        if (now - shootSequenceStartSec >= HybridShootCommandConstants.PostRetractWaitSec) {
          superStructure.setIntakeMode(IntakeMode.OFF);
          shootSequence = ShootSequence.COMPLETE;
        }
      }
      default -> {}
    }
  }

  private void abortShootSequence() {
    indexer.stop();
    shootSequence = ShootSequence.FEEDING;
    shootSequenceStartSec = 0.0;
  }

  private void cleanup() {
    if (superStructure.getShootPhase() == ShootPhase.SHOOT) {
      abortShootSequence();
    }
    hood.setIdle();
    shooter.stop();
    superStructure.setShootPhase(ShootPhase.OFF);
  }

  private void applyOperatorOffsets(ShotPlan plan) {
    if (operatorBPressed.getAsBoolean()) {
      rpsOffset = -1.0;
    }
    if (operatorAPressed.getAsBoolean()) {
      rpsOffset = -2.0;
    }
    if (operatorXPressed.getAsBoolean()) {
      rpsOffset = 1.0;
    }
    if (operatorYPressed.getAsBoolean()) {
      rpsOffset = 2.0;
    }
    plan.hoodCompDegs =
        ImprovedCommandXboxController.applyInputCurve(operatorHoodAxisSupplier.getAsDouble())
            * HybridShootCommandConstants.HoodCompRangeDegs;
    plan.headingCompDegs =
        ImprovedCommandXboxController.applyInputCurve(operatorAimHeadingAxisSupplier.getAsDouble())
            * HybridShootCommandConstants.AimHeadingCompRangeDegs;
    plan.rpsOffset = rpsOffset;
  }

  private void applyMechanisms(ShotPlan plan, boolean lockedDrive) {
    hood.setAutoSetpoint(plan.hoodDegs);
    hood.setOperatorInputScalar(plan.hoodCompDegs / HybridShootCommandConstants.HoodCompRangeDegs);
    shooter.setVelocityRps(plan.shooterRps + plan.rpsOffset);

    if (lockedDrive) {
      drive.driveAutoAimLocked(
          plan.heading,
          plan.effectiveShootHeadingCompDegs(),
          Math.abs(plan.shootHeadingFineTuneDegs)
              > HybridShootCommandConstants.ShootHeadingFineTuneDeadband);
    } else {
      drive.driveAutoAim(
          driverXSupplier, driverYSupplier, () -> plan.heading, plan.headingCompDegs);
    }
  }

  private boolean isAtTargetAngle(ShotPlan plan) {
    Rotation2d desired = plan.heading.plus(Rotation2d.fromDegrees(plan.headingCompDegs));
    return MathUtil.isNear(
        desired.getDegrees(),
        drive.getRotation().getDegrees(),
        HybridShootCommandConstants.HeadingToleranceDegs);
  }

  private boolean isAtTargetHoodDegs(ShotPlan plan) {
    return MathUtil.isNear(
        plan.hoodDegs + plan.hoodCompDegs,
        hood.getPositionDegs(),
        HybridShootCommandConstants.HoodToleranceDegs);
  }

  private boolean isAtTargetShootVelocity(ShotPlan plan) {
    return MathUtil.isNear(
        plan.shooterRps + plan.rpsOffset,
        shooter.getVelocityRps(),
        HybridShootCommandConstants.ShooterToleranceRps);
  }

  private void logShot(ShotPlan plan, boolean ready, boolean fire) {
    Logger.recordOutput("Cmds/HybridShoot/ControlMode", superStructure.getControlMode().toString());
    Logger.recordOutput("Cmds/HybridShoot/ShootPhase", superStructure.getShootPhase().toString());
    Logger.recordOutput("Cmds/HybridShoot/ShootSequence", shootSequence.toString());
    Logger.recordOutput("Cmds/HybridShoot/Fire", fire);
    Logger.recordOutput("Cmds/HybridShoot/Ready", ready);
    Logger.recordOutput("Cmds/HybridShoot/UsesMotionSolver", plan.usesMotionSolver);
    Logger.recordOutput("Cmds/HybridShoot/DistanceMeters", plan.distanceMeters);
    Logger.recordOutput("Cmds/HybridShoot/TargetHeadingDegs", plan.heading.getDegrees());
    Logger.recordOutput("Cmds/HybridShoot/HoodDegs", plan.hoodDegs);
    Logger.recordOutput("Cmds/HybridShoot/TargetRps", plan.shooterRps);
    Logger.recordOutput("Cmds/HybridShoot/RpsOffset", plan.rpsOffset);
    Logger.recordOutput("Cmds/HybridShoot/AtAngle", isAtTargetAngle(plan));
    Logger.recordOutput("Cmds/HybridShoot/AtHood", isAtTargetHoodDegs(plan));
    Logger.recordOutput("Cmds/HybridShoot/AtShooter", isAtTargetShootVelocity(plan));
    if (plan.virtualTarget != null) {
      Logger.recordOutput(
          "Cmds/HybridShoot/VirtualTarget", new Pose2d(plan.virtualTarget, Rotation2d.kZero));
    }
  }

  static final class ShotPlan {
    final boolean usesMotionSolver;
    final Translation2d target;
    final double distanceMeters;
    final Rotation2d heading;
    final double hoodDegs;
    final double shooterRps;
    final Translation2d virtualTarget;
    double hoodCompDegs;
    double headingCompDegs;
    double rpsOffset;
    double shootHeadingFineTuneDegs;

    ShotPlan(
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

    double effectiveShootHeadingCompDegs() {
      if (Math.abs(shootHeadingFineTuneDegs)
          <= HybridShootCommandConstants.ShootHeadingFineTuneDeadband) {
        return headingCompDegs;
      }
      return headingCompDegs
          + shootHeadingFineTuneDegs * HybridShootCommandConstants.AimHeadingCompRangeDegs;
    }
  }

  static final class ShotPlanner {
    private ShotPlanner() {}

    static ShotPlan plan(ControlMode mode, CommandSwerveDrivetrain drive) {
      Translation2d shooterPos = drive.getShooterWorldPosition();
      if (mode == ControlMode.SCORE) {
        Translation2d hub = CommandSwerveDrivetrain.getAllianceHubCenter();
        double distanceMeters = shooterPos.getDistance(hub);
        return new ShotPlan(
            false,
            hub,
            distanceMeters,
            hub.minus(shooterPos).getAngle(),
            ProjectileCalculator.getHoodTargetDegs(distanceMeters),
            ProjectileCalculator.getShooterTargetVelocity(distanceMeters),
            null);
      }

      Translation2d passTarget = resolvePassTarget(shooterPos);
      ShotSolution sol =
          ProjectileCalculator.solve(shooterPos, passTarget, drive.getFieldVelocity());
      return new ShotPlan(
          true,
          passTarget,
          sol.lookaheadDistance(),
          sol.aimAngle(),
          sol.hoodAngleDeg(),
          sol.shooterRps(),
          sol.virtualTarget());
    }

    private static Translation2d resolvePassTarget(Translation2d shooterPos) {
      boolean isBlue =
          DriverStation.getAlliance().isPresent()
              && DriverStation.getAlliance().get() == Alliance.Blue;
      double passX = isBlue ? 0.5 : FieldConstants.fieldLength - 0.5;
      double leftBumpCenterY =
          (FieldConstants.LinesHorizontal.leftBumpStart
                  + FieldConstants.LinesHorizontal.leftBumpEnd)
                  / 2.0
              + 1.0;
      double rightBumpCenterY =
          (FieldConstants.LinesHorizontal.rightBumpStart
                  + FieldConstants.LinesHorizontal.rightBumpEnd)
                  / 2.0
              - 1.0;
      double fieldCenterY = FieldConstants.fieldWidth / 2.0;
      return shooterPos.getY() > fieldCenterY
          ? new Translation2d(passX, leftBumpCenterY)
          : new Translation2d(passX, rightBumpCenterY);
    }
  }
}
