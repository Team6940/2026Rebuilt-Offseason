package frc.robot.commands;

import static frc.robot.Constants.HybridShootConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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

/** Driver Y: pass lane with full auto-aim during SHOOT. */
public class HybridPassCommand extends Command {

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

  public HybridPassCommand(CommandSwerveDrivetrain drive, Button shootButton) {
    this.drive = drive;
    this.shootButton = shootButton;
    addRequirements(drive, hood, shooter, indexer);
  }

  public HybridPassCommand(Button shootButton) {
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
    transitionTo(ShootPhase.AIM);
  }

  @Override
  public void execute() {
    applyOperatorAdjustments();

    ShotPlan plan = computeShotPlan();

    double finalHoodDegs = plan.hoodDegs + hoodCompDegs;
    double finalShooterRps = plan.shooterRps + rpsOffset;
    Rotation2d finalHeading = plan.heading.plus(Rotation2d.fromDegrees(headingCompDegs));

    if (superStructure.getShootPhase() == ShootPhase.SHOOT) {
      shootHeadingFineTuneDegs =
          ImprovedCommandXboxController.applyInputCurve(-operatorController.getRightX());
      if (Math.abs(shootHeadingFineTuneDegs) > ShootHeadingFineTuneDeadband) {
        finalHeading =
            finalHeading.plus(
                Rotation2d.fromDegrees(shootHeadingFineTuneDegs * AimHeadingCompRangeDegs));
      }
    }

    boolean ready = isReady(finalHoodDegs, finalShooterRps, finalHeading);
    boolean fire = driverController.getButton(shootButton);

    hood.setAutoSetpoint(finalHoodDegs);
    shooter.setVelocityRps(finalShooterRps);

    Rotation2d aimHeading = finalHeading;

    switch (superStructure.getShootPhase()) {
      case AIM -> {
        drive.driveAutoAim(
            () -> -driverController.getLeftY(),
            () -> -driverController.getLeftX(),
            () -> aimHeading);
        if (ready) {
          transitionTo(ShootPhase.READY);
        }
      }
      case READY -> {
        drive.driveAutoAim(
            () -> -driverController.getLeftY(),
            () -> -driverController.getLeftX(),
            () -> aimHeading);
        if (!ready) {
          transitionTo(ShootPhase.AIM);
        } else if (fire) {
          transitionTo(ShootPhase.SHOOT);
        }
      }
      case SHOOT -> {
        drive.driveAutoAim(
            () -> -driverController.getLeftY(),
            () -> -driverController.getLeftX(),
            () -> aimHeading);

        runShootSequence(finalHoodDegs, ready, fire);
        if (!fire) {
          transitionTo(ready ? ShootPhase.READY : ShootPhase.AIM);
        }
      }
      default -> transitionTo(ShootPhase.AIM);
    }

    log(plan, finalHoodDegs, finalShooterRps, finalHeading, ready, fire);
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
    return ProjectileCalculator.planPass(drive.getShooterWorldPosition(), drive.getFieldVelocity());
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

  private void runShootSequence(double finalHoodDegs, boolean ready, boolean fire) {
    double now = Timer.getFPGATimestamp();
    switch (shootSequence) {
      case FEEDING -> {
        if (Constants.currentMode == Constants.Mode.SIM
            && fire
            && now - lastSimVolleySec >= FieldSimulationConstants.DumperVolleyPeriodSec
            && ready) {
          shooter.simulateLaunch(90. - finalHoodDegs);
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

  private boolean isReady(double hoodDegs, double shooterRps, Rotation2d heading) {
    return isAtTargetAngle(heading)
        && isAtTargetHood(hoodDegs)
        && isAtTargetShooter(shooterRps);
  }

  private boolean isAtTargetAngle(Rotation2d desired) {
    return MathUtil.isNear(
        desired.getDegrees(), drive.getRotation().getDegrees(), HeadingToleranceDegs);
  }

  private boolean isAtTargetHood(double hoodDegs) {
    return MathUtil.isNear(hoodDegs, hood.getPositionDegs(), HoodToleranceDegs);
  }

  private boolean isAtTargetShooter(double shooterRps) {
    return MathUtil.isNear(shooterRps, shooter.getVelocityRps(), ShooterToleranceRps);
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
    Logger.recordOutput("Cmds/HybridPass/StateTransition", current + "->" + next);
  }

  private void log(
      ShotPlan plan,
      double finalHoodDegs,
      double finalShooterRps,
      Rotation2d finalHeading,
      boolean ready,
      boolean fire) {
    Logger.recordOutput("Cmds/HybridPass/ControlMode", superStructure.getControlMode().toString());
    Logger.recordOutput("Cmds/HybridPass/ShootPhase", superStructure.getShootPhase().toString());
    Logger.recordOutput("Cmds/HybridPass/ShootSequence", shootSequence.toString());
    Logger.recordOutput("Cmds/HybridPass/Fire", fire);
    Logger.recordOutput("Cmds/HybridPass/Ready", ready);
    Logger.recordOutput("Cmds/HybridPass/UsesMotionSolver", plan.usesMotionSolver);
    Logger.recordOutput("Cmds/HybridPass/DistanceMeters", plan.distanceMeters);
    Logger.recordOutput("Cmds/HybridPass/TargetHeadingDegs", finalHeading.getDegrees());
    Logger.recordOutput("Cmds/HybridPass/HoodDegs", finalHoodDegs);
    Logger.recordOutput("Cmds/HybridPass/TargetRps", finalShooterRps);
    Logger.recordOutput("Cmds/HybridPass/RpsOffset", rpsOffset);
    Logger.recordOutput("Cmds/HybridPass/AtAngle", isAtTargetAngle(finalHeading));
    Logger.recordOutput("Cmds/HybridPass/AtHood", isAtTargetHood(finalHoodDegs));
    Logger.recordOutput("Cmds/HybridPass/AtShooter", isAtTargetShooter(finalShooterRps));
    if (plan.virtualTarget != null) {
      Logger.recordOutput(
          "Cmds/HybridPass/VirtualTarget", new Pose2d(plan.virtualTarget, Rotation2d.kZero));
    }
  }
}
