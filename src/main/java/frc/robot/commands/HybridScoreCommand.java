package frc.robot.commands;

import static frc.robot.Constants.HybridShootConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.FieldSimulationConstants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Chassis.CommandSwerveDrivetrain;
import frc.robot.subsystems.Hood.HoodSubsystem;
import frc.robot.subsystems.ImprovedCommandXboxController;
import frc.robot.subsystems.ImprovedCommandXboxController.Button;
import frc.robot.subsystems.Indexer.IndexerSubsystem;
import frc.robot.subsystems.Shooter.ShooterSubsystem;
import frc.robot.subsystems.SuperStructure;
import frc.robot.subsystems.SuperStructure.DriveMode;
import frc.robot.subsystems.SuperStructure.IntakeMode;
import frc.robot.subsystems.SuperStructure.ShootPhase;
import frc.robot.util.ProjectileCalculator;
import frc.robot.util.ProjectileCalculator.ShotPlan;
import org.littletonrobotics.junction.Logger;

/** Driver RB: hub score with locked chassis during SHOOT. */
public class HybridScoreCommand extends Command {

  private enum ShootSequence {
    FEEDING,
    RETRACT_WAIT,
    COMPLETE
  }

  private final CommandSwerveDrivetrain drive;
  private final Button shootButton;
  private final ImprovedCommandXboxController driverController = RobotContainer.driverController;
  private final ImprovedCommandXboxController operatorController =
      RobotContainer.operatorController;

  private final HoodSubsystem hood = HoodSubsystem.getInstance();
  private final ShooterSubsystem shooter = ShooterSubsystem.getInstance();
  private final IndexerSubsystem indexer = IndexerSubsystem.getInstance();
  private final SuperStructure superStructure = SuperStructure.getInstance();

  private ShootSequence shootSequence = ShootSequence.FEEDING;
  private double shootSequenceStartSec = 0.0;
  private double rpsOffset = 0.0;
  private double hoodCompDegs = 0.0;
  private double headingCompDegs = 0.0;
  private double shootHeadingFineTuneDegs = 0.0;
  private double lastSimVolleySec = 0.0;

  public HybridScoreCommand(CommandSwerveDrivetrain drive, Button shootButton) {
    this.drive = drive;
    this.shootButton = shootButton;
    addRequirements(drive, hood, shooter, indexer);
  }

  public HybridScoreCommand(Button shootButton) {
    this(CommandSwerveDrivetrain.getInstance(), shootButton);
  }

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
    transitionTo(ShootPhase.AIM);
  }

  @Override
  public void execute() {
    applyOperatorAdjustments();

    ShotPlan plan = computeShotPlan();
    boolean ready = isReady(plan);
    boolean fire = driverController.getButton(shootButton);

    hood.setAutoSetpoint(plan.hoodDegs);
    hood.setOperatorInputScalar(hoodCompDegs / HoodCompRangeDegs);
    shooter.setVelocityRps(plan.shooterRps + rpsOffset);

    switch (superStructure.getShootPhase()) {
      case AIM -> {
        drive.driveAutoAim(
            () -> -driverController.getLeftY(),
            () -> -driverController.getLeftX(),
            () -> plan.heading,
            headingCompDegs);
        if (ready) {
          transitionTo(ShootPhase.READY);
        }
      }
      case READY -> {
        drive.driveAutoAim(
            () -> -driverController.getLeftY(),
            () -> -driverController.getLeftX(),
            () -> plan.heading,
            headingCompDegs);
        if (!ready) {
          transitionTo(ShootPhase.AIM);
        } else if (fire) {
          transitionTo(ShootPhase.SHOOT);
        }
      }
      case SHOOT -> {
        shootHeadingFineTuneDegs =
            ImprovedCommandXboxController.applyInputCurve(-operatorController.getRightX());
        double shootHeadingCompDegs = headingCompDegs;
        if (Math.abs(shootHeadingFineTuneDegs) > ShootHeadingFineTuneDeadband) {
          shootHeadingCompDegs += shootHeadingFineTuneDegs * AimHeadingCompRangeDegs;
        }
        Translation2d driverInput =
            new Translation2d(-driverController.getLeftX(), -driverController.getLeftY());

        drive.driveAutoAimWithSpeedLimitorLocked(
            () -> -driverController.getLeftY(),
            () -> -driverController.getLeftX(),
            plan.heading,
            shootHeadingCompDegs,
            Math.abs(shootHeadingFineTuneDegs) > ShootHeadingFineTuneDeadband,
            driverInput.getNorm() > DriverTranslationFineTuneDeadband);

        runShootSequence(plan, ready, fire);
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
    if (superStructure.getShootPhase() == ShootPhase.SHOOT) {
      indexer.stop();
      shootSequence = ShootSequence.FEEDING;
      shootSequenceStartSec = 0.0;
    }
    hood.setIdle();
    shooter.stop();
    drive.releaseAutoAimCurrentLimits();
    superStructure.setShootPhase(ShootPhase.OFF);
    if (!interrupted) {
      superStructure.claimDriveMode(DriveMode.MANUAL);
    }
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  private ShotPlan computeShotPlan() {
    var shooterPos = drive.getShooterWorldPosition();
    return ProjectileCalculator.planScore(
        shooterPos, CommandSwerveDrivetrain.getAllianceHubCenter(), drive.getFieldVelocity());
  }

  private void applyOperatorAdjustments() {
    if (operatorController.getButtonPressed(Button.kB)) {
      rpsOffset = RpsOffsetB;
    }
    if (operatorController.getButtonPressed(Button.kA)) {
      rpsOffset = RpsOffsetA;
    }
    if (operatorController.getButtonPressed(Button.kX)) {
      rpsOffset = RpsOffsetX;
    }
    if (operatorController.getButtonPressed(Button.kY)) {
      rpsOffset = RpsOffsetY;
    }
    hoodCompDegs =
        ImprovedCommandXboxController.applyInputCurve(-operatorController.getLeftY())
            * HoodCompRangeDegs;
    headingCompDegs =
        ImprovedCommandXboxController.applyInputCurve(-operatorController.getRightX())
            * AimHeadingCompRangeDegs;
  }

  private void runShootSequence(ShotPlan plan, boolean ready, boolean fire) {
    double now = Timer.getFPGATimestamp();
    switch (shootSequence) {
      case FEEDING -> {
        if (Constants.currentMode == Constants.Mode.SIM
            && fire
            && now - lastSimVolleySec >= FieldSimulationConstants.DumperVolleyPeriodSec
            && ready) {
          shooter.simulateLaunch(90. - (plan.hoodDegs + hoodCompDegs));
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
  }

  private boolean isReady(ShotPlan plan) {
    return isAtTargetAngle(plan) && isAtTargetHood(plan) && isAtTargetShooter(plan);
  }

  private boolean isAtTargetAngle(ShotPlan plan) {
    Rotation2d desired = plan.heading.plus(Rotation2d.fromDegrees(headingCompDegs));
    return MathUtil.isNear(
        desired.getDegrees(), drive.getRotation().getDegrees(), HeadingToleranceDegs);
  }

  private boolean isAtTargetHood(ShotPlan plan) {
    return MathUtil.isNear(plan.hoodDegs + hoodCompDegs, hood.getPositionDegs(), HoodToleranceDegs);
  }

  private boolean isAtTargetShooter(ShotPlan plan) {
    return MathUtil.isNear(
        plan.shooterRps + rpsOffset, shooter.getVelocityRps(), ShooterToleranceRps);
  }

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
      double now = Timer.getFPGATimestamp();
      shootSequenceStartSec = now;
      lastSimVolleySec = now - FieldSimulationConstants.DumperVolleyPeriodSec;
      indexer.feed();
    }
    superStructure.setShootPhase(next);
    Logger.recordOutput("Cmds/HybridScore/StateTransition", current + "->" + next);
  }

  private void log(ShotPlan plan, boolean ready, boolean fire) {
    Logger.recordOutput("Cmds/HybridScore/ControlMode", superStructure.getControlMode().toString());
    Logger.recordOutput("Cmds/HybridScore/ShootPhase", superStructure.getShootPhase().toString());
    Logger.recordOutput("Cmds/HybridScore/ShootSequence", shootSequence.toString());
    Logger.recordOutput("Cmds/HybridScore/Fire", fire);
    Logger.recordOutput("Cmds/HybridScore/Ready", ready);
    Logger.recordOutput("Cmds/HybridScore/UsesMotionSolver", plan.usesMotionSolver);
    Logger.recordOutput("Cmds/HybridScore/DistanceMeters", plan.distanceMeters);
    Logger.recordOutput("Cmds/HybridScore/TargetHeadingDegs", plan.heading.getDegrees());
    Logger.recordOutput("Cmds/HybridScore/HoodDegs", plan.hoodDegs);
    Logger.recordOutput("Cmds/HybridScore/TargetRps", plan.shooterRps);
    Logger.recordOutput("Cmds/HybridScore/RpsOffset", rpsOffset);
    Logger.recordOutput("Cmds/HybridScore/AtAngle", isAtTargetAngle(plan));
    Logger.recordOutput("Cmds/HybridScore/AtHood", isAtTargetHood(plan));
    Logger.recordOutput("Cmds/HybridScore/AtShooter", isAtTargetShooter(plan));
    if (plan.virtualTarget != null) {
      Logger.recordOutput(
          "Cmds/HybridScore/VirtualTarget", new Pose2d(plan.virtualTarget, Rotation2d.kZero));
    }
  }
}
