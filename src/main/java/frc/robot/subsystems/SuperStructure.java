package frc.robot.subsystems;

import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveRequest.FieldCentric;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.DriveDefaultCommand;
import frc.robot.commands.DriveHybridTrenchCommand;
import frc.robot.commands.HeatupCommand;
import frc.robot.commands.HybridPassCommand;
import frc.robot.commands.HybridScoreCommand;
import frc.robot.commands.IndexerDefaultCommand;
import frc.robot.commands.IntakeDefaultCommand;
import frc.robot.commands.IntakeEmergencyOutCommand;
import frc.robot.commands.IntakeHybridCommand;
import frc.robot.commands.ManualShootCommand;
import frc.robot.commands.leds.LEDDefaultCommand;
import frc.robot.subsystems.Chassis.CommandSwerveDrivetrain;
import frc.robot.subsystems.Halo.LEDController;
import frc.robot.subsystems.ImprovedCommandXboxController;
import frc.robot.subsystems.ImprovedCommandXboxController.Button;
import frc.robot.subsystems.Intake.IntakeSubsystem;
import frc.robot.subsystems.Power.PowerMonitor;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

/**
 * Central coordinator for robot modes, subsystem access, and command getters.
 *
 * <p>Commands read and write {@link DriveMode}, {@link ControlMode}, {@link ShootPhase}, and {@link
 * IntakeMode} here. Button bindings live in {@link frc.robot.RobotContainer}.
 */
public class SuperStructure extends SubsystemBase {
  private static SuperStructure instance;

  public static SuperStructure getInstance() {
    if (instance == null) {
      instance = new SuperStructure();
    }
    return instance;
  }

  public enum DriveMode {
    AUTO_AIM,
    HYBRID_TRENCH,
    HYBRID_INTAKE_DRIVE,
    MANUAL
  }

  /** Hub score, pass lane, or SmartDashboard lookup-table tuning. */
  public enum ControlMode {
    SCORE,
    PASS,
    MANUAL
  }

  /** Hybrid shooting sequence state. */
  public enum ShootPhase {
    OFF,
    HEATUP,
    AIM,
    READY,
    SHOOT
  }

  public enum IntakeMode {
    INTAKE,
    HYBRID,
    RETRACTED,
    OFF,
    REVERSE,
    MID
  }

  /** Chassis current-limit profile. */
  public enum ChassisMode {
    /** TunerConstants defaults (drive 60A supply/60A stator, steer 50A supply/50A stator). */
    NORMAL,
    /** Reduced drive current to free battery for shooter. */
    SHOOTING,
    /** No current limits at all. */
    ATTACKMODE
  }

  private final LoggedNetworkNumber manualShootVelocityRps =
      new LoggedNetworkNumber("SmartDashboard/ShootVelocity", 0.0);
  private final LoggedNetworkNumber manualHoodDegs =
      new LoggedNetworkNumber("SmartDashboard/HoodDegs", 0.0);

  private DriveMode driveMode = DriveMode.MANUAL;
  private ControlMode controlMode = ControlMode.SCORE;
  private ShootPhase shootPhase = ShootPhase.OFF;
  private IntakeMode intakeMode = IntakeMode.OFF;
  private ChassisMode chassisMode = ChassisMode.NORMAL;
  private ChassisMode previousChassisMode = ChassisMode.NORMAL;
  private boolean attackModeActive = false;
  private final Timer attackModeTimer = new Timer();
  private boolean fieldCentricEnbaled = true;

  /** AIM-phase readiness flags for LED progress visualization. */
  private boolean atAngle = false;

  private boolean atHood = false;
  private boolean atShooter = false;

  public LEDController getLEDs() {
    return LEDController.getInstance();
  }

  private SuperStructure() {
    // Ensure PowerMonitor is instantiated and scheduled
    PowerMonitor.getInstance();
  }

  public CommandSwerveDrivetrain getDrive() {
    return CommandSwerveDrivetrain.getInstance();
  }

  public IntakeSubsystem getIntake() {
    return IntakeSubsystem.getInstance();
  }

  public DoubleSupplier getManualShootVelocityRps() {
    return manualShootVelocityRps::get;
  }

  public DoubleSupplier getManualHoodDegs() {
    return manualHoodDegs::get;
  }

  public void claimDriveMode(DriveMode mode) {
    driveMode = mode;
  }

  public void resetAllModes() {
    fieldCentricEnbaled = true;
    driveMode = DriveMode.MANUAL;
    controlMode = ControlMode.SCORE;
    shootPhase = ShootPhase.OFF;
    intakeMode = IntakeMode.OFF;
    chassisMode = ChassisMode.NORMAL;
    previousChassisMode = ChassisMode.NORMAL;
    attackModeActive = false;
    attackModeTimer.reset();
    CommandScheduler.getInstance().cancel(getIntake().getCurrentCommand());
  }

  public void setControlMode(ControlMode mode) {
    controlMode = mode;
  }

  public void setShootPhase(ShootPhase phase) {
    shootPhase = phase;
  }

  public void setIntakeMode(IntakeMode mode) {
    intakeMode = mode;
  }

  public void toggleControlMode() {
    controlMode =
        switch (controlMode) {
          case SCORE -> ControlMode.PASS;
          case PASS -> ControlMode.MANUAL;
          case MANUAL -> ControlMode.SCORE;
        };
  }

  /** Driver LB: INTAKE/HYBRID to RETRACTED to OFF. */
  public void cycleIntakeRetractState() {
    switch (intakeMode) {
      case INTAKE, HYBRID -> setIntakeMode(IntakeMode.RETRACTED);
      case RETRACTED -> setIntakeMode(IntakeMode.OFF);
      default -> {}
    }
  }

  public void stopDriveWithX() {
    getDrive().stopWithX();
  }

  public void resetRobotHeading() {
    CommandSwerveDrivetrain drive = getDrive();
    drive.setPose(new Pose2d(drive.getPose().getTranslation(), new Rotation2d()));
  }

  public DriveMode getDriveMode() {
    return driveMode;
  }

  public ControlMode getControlMode() {
    return controlMode;
  }

  public ShootPhase getShootPhase() {
    return shootPhase;
  }

  public IntakeMode getIntakeMode() {
    return intakeMode;
  }

  public boolean getFieldCentricEnbaled() {
    return fieldCentricEnbaled;
  }

  public void toggleFieldCentricEnabled() {
    fieldCentricEnbaled = fieldCentricEnbaled ? false : true;
  }

  /** Switches chassis current-limit profile. Saves previous mode when entering SHOOTING. */
  public void setChassisMode(ChassisMode mode) {
    if (chassisMode == mode) {
      return;
    }
    if (mode == ChassisMode.SHOOTING) {
      previousChassisMode = chassisMode;
    }
    chassisMode = mode;
    getDrive().applyChassisModeLimits(mode);
  }

  /** Restores the mode that was active before SHOOTING. */
  public void restoreChassisMode() {
    ChassisMode restore = previousChassisMode;
    chassisMode = restore;
    getDrive().applyChassisModeLimits(restore);
  }

  public ChassisMode getChassisMode() {
    return chassisMode;
  }

  public void enableAttackMode() {
    if (attackModeActive) {
      return;
    }
    attackModeActive = true;
    attackModeTimer.reset();
    attackModeTimer.start();
    setChassisMode(ChassisMode.ATTACKMODE);
  }

  public void disableAttackMode() {
    if (!attackModeActive) {
      return;
    }
    attackModeActive = false;
    attackModeTimer.stop();
    attackModeTimer.reset();
    setChassisMode(ChassisMode.NORMAL);
  }

  public boolean isAttackModeActive() {
    return attackModeActive;
  }

  public void setAimReadiness(boolean atAngle, boolean atHood, boolean atShooter) {
    this.atAngle = atAngle;
    this.atHood = atHood;
    this.atShooter = atShooter;
  }

  public boolean isAtAngle() {
    return atAngle;
  }

  public boolean isAtHood() {
    return atHood;
  }

  public boolean isAtShooter() {
    return atShooter;
  }

  /** Returns 0-3 count of ready subsystems. */
  public int getAimReadyCount() {
    int count = 0;
    if (atAngle) count++;
    if (atHood) count++;
    if (atShooter) count++;
    return count;
  }

  public Command getDefaultDriveCommand(
      DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier omegaSupplier) {
    claimDriveMode(DriveMode.MANUAL);
    CommandSwerveDrivetrain drive = getDrive();
    return new DriveDefaultCommand(drive, xSupplier, ySupplier, omegaSupplier);
  }

  /**
   * Sets {@link ControlMode} and claims {@link DriveMode#AUTO_AIM}, then returns the matching shoot
   * command.
   */
  public Command getShootCommand(ControlMode mode, Button shootButton) {
    setControlMode(mode);
    claimDriveMode(DriveMode.AUTO_AIM);
    return switch (mode) {
      case SCORE -> new HybridScoreCommand(getDrive(), shootButton);
      case PASS -> new HybridPassCommand(getDrive(), shootButton);
      case MANUAL -> new ManualShootCommand(getDrive(), shootButton);
    };
  }

  public HeatupCommand getHeatupCommand() {
    return new HeatupCommand();
  }

  public Command getHybridTrenchCommand() {
    claimDriveMode(DriveMode.HYBRID_TRENCH);
    return new DriveHybridTrenchCommand(getDrive());
  }

  public Command getHybridIntakeCommand(
      ImprovedCommandXboxController controller,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier) {
    claimDriveMode(DriveMode.HYBRID_INTAKE_DRIVE);
    return new IntakeHybridCommand(getDrive(), controller, xSupplier, ySupplier, omegaSupplier);
  }

  public IntakeDefaultCommand getIntakeDefaultCommand() {
    return new IntakeDefaultCommand();
  }

  public IntakeEmergencyOutCommand getIntakeEmergencyOutCommand() {
    return new IntakeEmergencyOutCommand();
  }

  public LEDDefaultCommand getLEDDefaultCommand() {
    return new LEDDefaultCommand(getLEDs());
  }

  public IndexerDefaultCommand getIndexerDefaultCommand() {
    return new IndexerDefaultCommand();
  }

  @Override
  public void periodic() {
    if (attackModeActive
        && attackModeTimer.hasElapsed(Constants.DriveConstants.AttackModeTimeoutSec)) {
      disableAttackMode();
    }

    Logger.recordOutput("SuperStructure/DriveMode", driveMode);
    Logger.recordOutput("SuperStructure/ControlMode", controlMode);
    Logger.recordOutput("SuperStructure/ShootPhase", shootPhase);
    Logger.recordOutput("SuperStructure/IntakeMode", intakeMode);
    Logger.recordOutput("SuperStructure/ChassisMode", chassisMode.name());
    Logger.recordOutput("SuperStructure/AttackModeActive", attackModeActive);
    Logger.recordOutput("SuperStructure/AttackModeTimerSec", attackModeTimer.get());
    Logger.recordOutput("SuperStructure/ManualShootVelocityRps", manualShootVelocityRps.get());
    Logger.recordOutput("SuperStructure/ManualHoodDegs", manualHoodDegs.get());
    Logger.recordOutput("SuperStructure/FieldCentricDrive", fieldCentricEnbaled);
  }
}
