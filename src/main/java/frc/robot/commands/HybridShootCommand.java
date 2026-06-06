package frc.robot.commands;

import static frc.robot.Constants.HybridShootConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.FieldSimulationConstants;
import frc.robot.subsystems.Chassis.CommandSwerveDrivetrain;
import frc.robot.subsystems.Hood.HoodSubsystem;
import frc.robot.subsystems.ImprovedCommandXboxController;
import frc.robot.subsystems.Indexer.IndexerSubsystem;
import frc.robot.subsystems.Shooter.ShooterSubsystem;
import frc.robot.subsystems.SuperStructure;
import frc.robot.subsystems.SuperStructure.DriveMode;
import frc.robot.subsystems.SuperStructure.ControlMode;
import frc.robot.subsystems.SuperStructure.IntakeMode;
import frc.robot.subsystems.SuperStructure.ShootPhase;
import frc.robot.util.ProjectileCalculator;
import frc.robot.util.ProjectileCalculator.ShotPlan;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

/**
 * Hybrid shoot: auto-aim chassis + hood + shooter while the driver holds aim.
 *
 * <p><b>Bindings</b> ({@link frc.robot.RobotContainer}): driver RB = hub (SCORE), Y = pass lane
 * (PASS); driver RT = fire; operator left Y = hood trim, right X = heading trim; operator B/A/X/Y =
 * RPS offset steps.
 *
 * <p><b>{@link ShootPhase}</b> (while RB or Y held):
 *
 * <pre>
 * AIM ──(heading + hood + shooter in tolerance)──► READY ──(RT)──► SHOOT
 *  ▲                      │                         │
 *  └── lost ready ────────┘                         │
 *  ▲◄──────────── RT released ───────────────────────┘
 * </pre>
 *
 * <p><b>SHOOT sub-sequence</b> ({@link ShootSequence}): feed indexer → retract intake → idle. Ballistics
 * in {@link ProjectileCalculator}; tuning in {@link frc.robot.Constants.HybridShootConstants}; spin-up-only
 * is {@link HeatupCommand}.
 */
public class HybridShootCommand extends Command {

  /** Steps inside {@link ShootPhase#SHOOT} after RT starts a shot. */
  private enum ShootSequence {
    FEEDING,
    RETRACT_WAIT,
    COMPLETE
  }

  // --- Injected inputs ---
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

  // --- Subsystems ---
  private final HoodSubsystem hood = HoodSubsystem.getInstance();
  private final ShooterSubsystem shooter = ShooterSubsystem.getInstance();
  private final IndexerSubsystem indexer = IndexerSubsystem.getInstance();
  private final SuperStructure superStructure = SuperStructure.getInstance();

  // --- Mutable state (reset in initialize / transitionTo) ---
  private ShootSequence shootSequence = ShootSequence.FEEDING;
  private double shootSequenceStartSec = 0.0;
  private double rpsOffset = 0.0;
  private double hoodCompDegs = 0.0;
  private double headingCompDegs = 0.0;
  /** Right stick during SHOOT only; overrides headingCompDegs when past deadband. */
  private double shootHeadingFineTuneDegs = 0.0;
  /** SIM: last full-width dumper volley timestamp. */
  private double lastSimVolleySec = 0.0;

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

  // --- Command lifecycle ---

  @Override
  public void initialize() {
    shootSequence = ShootSequence.FEEDING;
    shootSequenceStartSec = 0.0;
    rpsOffset = 0.0;
    hoodCompDegs = 0.0;
    headingCompDegs = 0.0;
    shootHeadingFineTuneDegs = 0.0;
    lastSimVolleySec = 0.0;
    hood.setOperatorInputScalar(0.0);
    superStructure.claimDriveMode(DriveMode.AUTO_AIM);
    transitionTo(ShootPhase.AIM);
  }

  @Override
  public void execute() {
    // SCORE vs PASS from held aim button
    updateControlMode();
    // Hood / heading trims + RPS offset steps (operator)
    applyOperatorAdjustments();

    ShotPlan plan = computeShotPlan();
    boolean ready = isReady(plan);
    boolean fire = fireSupplier.getAsBoolean();

    // Hood + shooter run in every ShootPhase; drive mode differs below
    hood.setAutoSetpoint(plan.hoodDegs);
    hood.setOperatorInputScalar(hoodCompDegs / HoodCompRangeDegs);
    shooter.setVelocityRps(plan.shooterRps + rpsOffset);

    switch (superStructure.getShootPhase()) {
      case AIM -> {
        // Driver translation + auto-rotate toward target
        drive.driveAutoAim(
            driverXSupplier, driverYSupplier, () -> plan.heading, headingCompDegs);
        if (ready) {
          transitionTo(ShootPhase.READY);
        }
      }
      case READY -> {
        drive.driveAutoAim(
            driverXSupplier, driverYSupplier, () -> plan.heading, headingCompDegs);
        if (!ready) {
          transitionTo(ShootPhase.AIM);
        } else if (fire) {
          transitionTo(ShootPhase.SHOOT);
        }
      }
      case SHOOT -> {
        // Locked chassis; operator right stick fine-tunes heading
        shootHeadingFineTuneDegs =
            ImprovedCommandXboxController.applyInputCurve(
                operatorAimHeadingAxisSupplier.getAsDouble());
        double shootHeadingCompDegs = headingCompDegs;
        if (Math.abs(shootHeadingFineTuneDegs) > ShootHeadingFineTuneDeadband) {
          shootHeadingCompDegs +=
              shootHeadingFineTuneDegs * AimHeadingCompRangeDegs;
        }
        if (ControlMode.SCORE.equals(superStructure.getControlMode())) {
          drive.driveAutoAimLocked(
            plan.heading,
            shootHeadingCompDegs,
            Math.abs(shootHeadingFineTuneDegs) > ShootHeadingFineTuneDeadband);
        } else {
          drive.driveAutoAim(driverXSupplier, driverYSupplier, ()->plan.heading, shootHeadingCompDegs);
        }
        // Feed ball, then command intake retract / release
        double now = Timer.getFPGATimestamp();
        switch (shootSequence) {
          case FEEDING -> {
            if (Constants.currentMode == Constants.Mode.SIM
                && fire
                && now - lastSimVolleySec >= FieldSimulationConstants.DumperVolleyPeriodSec) {
              shooter.simulateLaunch(plan.hoodDegs + hoodCompDegs);
              lastSimVolleySec = now;
            }
            if (now - shootSequenceStartSec >= FeedDurationSec) {
              superStructure.setIntakeMode(IntakeMode.RETRACTED);
              shootSequence = ShootSequence.RETRACT_WAIT;
              shootSequenceStartSec = now;
            }
          }
          case RETRACT_WAIT -> {
            if (now - shootSequenceStartSec >= PostRetractWaitSec) {
              superStructure.setIntakeMode(IntakeMode.OFF);
              shootSequence = ShootSequence.COMPLETE;
            }
          }
          default -> {}
        }

        if (!fire) {
          transitionTo(ready ? ShootPhase.READY : ShootPhase.AIM);
        }
      }
      default -> transitionTo(ShootPhase.AIM);
    }

    log(plan, ready, fire);
  }

  @Override
  public void end(boolean interrupted) {
    // Command cancelled while firing — same cleanup as leaving SHOOT in transitionTo
    if (superStructure.getShootPhase() == ShootPhase.SHOOT) {
      indexer.stop();
      shootSequence = ShootSequence.FEEDING;
      shootSequenceStartSec = 0.0;
    }
    hood.setIdle();
    shooter.stop();
    superStructure.setShootPhase(ShootPhase.OFF);
    if (!interrupted) {
      superStructure.claimDriveMode(DriveMode.MANUAL);
    }
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  // --- Control mode (SCORE / PASS) ---

  private void updateControlMode() {
    if (aimScoreSupplier.getAsBoolean()) {
      superStructure.setControlMode(ControlMode.SCORE);
    } else if (aimPassSupplier.getAsBoolean()) {
      superStructure.setControlMode(ControlMode.PASS);
    }
  }

  // --- Shot planning ---

  private ShotPlan computeShotPlan() {
    var shooterPos = drive.getShooterWorldPosition();
    if (superStructure.getControlMode() == ControlMode.SCORE) {
      return ProjectileCalculator.planScore(
          shooterPos, CommandSwerveDrivetrain.getAllianceHubCenter());
    }
    return ProjectileCalculator.planPass(shooterPos, drive.getFieldVelocity());
  }

  // --- Operator trims ---

  private void applyOperatorAdjustments() {
    if (operatorBPressed.getAsBoolean()) {
      rpsOffset = RpsOffsetB;
    }
    if (operatorAPressed.getAsBoolean()) {
      rpsOffset = RpsOffsetA;
    }
    if (operatorXPressed.getAsBoolean()) {
      rpsOffset = RpsOffsetX;
    }
    if (operatorYPressed.getAsBoolean()) {
      rpsOffset = RpsOffsetY;
    }
    hoodCompDegs =
        ImprovedCommandXboxController.applyInputCurve(operatorHoodAxisSupplier.getAsDouble())
            * HoodCompRangeDegs;
    headingCompDegs =
        ImprovedCommandXboxController.applyInputCurve(operatorAimHeadingAxisSupplier.getAsDouble())
            * AimHeadingCompRangeDegs;
  }

  // --- Ready checks (AIM → READY gate) ---

  private boolean isReady(ShotPlan plan) {
    return isAtTargetAngle(plan) && isAtTargetHood(plan) && isAtTargetShooter(plan);
  }

  private boolean isAtTargetAngle(ShotPlan plan) {
    Rotation2d desired = plan.heading.plus(Rotation2d.fromDegrees(headingCompDegs));
    return MathUtil.isNear(
        desired.getDegrees(), drive.getRotation().getDegrees(), HeadingToleranceDegs);
  }

  private boolean isAtTargetHood(ShotPlan plan) {
    return MathUtil.isNear(
        plan.hoodDegs + hoodCompDegs, hood.getPositionDegs(), HoodToleranceDegs);
  }

  private boolean isAtTargetShooter(ShotPlan plan) {
    return MathUtil.isNear(
        plan.shooterRps + rpsOffset, shooter.getVelocityRps(), ShooterToleranceRps);
  }

  // --- ShootPhase transitions (+ SHOOT indexer bookends) ---

  private void transitionTo(ShootPhase next) {
    ShootPhase current = superStructure.getShootPhase();
    if (current == next) {
      return;
    }
    if (current == ShootPhase.SHOOT) {
      indexer.stop();
      shootSequence = ShootSequence.FEEDING;
      shootSequenceStartSec = 0.0;
    }
    if (next == ShootPhase.SHOOT) {
      shootSequence = ShootSequence.FEEDING;
      shootSequenceStartSec = Timer.getFPGATimestamp();
      lastSimVolleySec = 0.0;
      indexer.feed();
    }
    superStructure.setShootPhase(next);
    Logger.recordOutput("Cmds/HybridShoot/StateTransition", current + "->" + next);
  }

  // --- AdvantageKit telemetry ---

  private void log(ShotPlan plan, boolean ready, boolean fire) {
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
    Logger.recordOutput("Cmds/HybridShoot/RpsOffset", rpsOffset);
    Logger.recordOutput("Cmds/HybridShoot/AtAngle", isAtTargetAngle(plan));
    Logger.recordOutput("Cmds/HybridShoot/AtHood", isAtTargetHood(plan));
    Logger.recordOutput("Cmds/HybridShoot/AtShooter", isAtTargetShooter(plan));
    if (plan.virtualTarget != null) {
      Logger.recordOutput(
          "Cmds/HybridShoot/VirtualTarget", new Pose2d(plan.virtualTarget, Rotation2d.kZero));
    }
  }
}
