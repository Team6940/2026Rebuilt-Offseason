package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.DriveHybridTrenchCommand;
import frc.robot.commands.HeatupCommand;
import frc.robot.commands.HybridPassCommand;
import frc.robot.commands.HybridScoreCommand;
import frc.robot.commands.IntakeEmergencyOutCommand;
import frc.robot.commands.IntakeDefaultCommand;
import frc.robot.commands.IntakeHybridCommand;
import frc.robot.commands.ManualShootCommand;
import frc.robot.commands.leds.LEDDefaultCommand;
import frc.robot.subsystems.Chassis.CommandSwerveDrivetrain;
import frc.robot.subsystems.ImprovedCommandXboxController;
import frc.robot.subsystems.ImprovedCommandXboxController.Button;
import frc.robot.subsystems.Intake.IntakeSubsystem;
import frc.robot.subsystems.leds.LEDController;
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

  private final LoggedNetworkNumber manualShootVelocityRps =
      new LoggedNetworkNumber("SmartDashboard/ShootVelocity", 0.0);
  private final LoggedNetworkNumber manualHoodDegs =
      new LoggedNetworkNumber("SmartDashboard/HoodDegs", 0.0);

  private DriveMode driveMode = DriveMode.MANUAL;
  private ControlMode controlMode = ControlMode.SCORE;
  private ShootPhase shootPhase = ShootPhase.OFF;
  private IntakeMode intakeMode = IntakeMode.OFF;

  public LEDController getLEDs() {
    return LEDController.getInstance();
  }

  private SuperStructure() {}

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
    claimDriveMode(DriveMode.MANUAL);
    setIntakeMode(IntakeMode.OFF);
    setShootPhase(ShootPhase.OFF);
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

  public Command getFieldCentricDriveCommand(
      DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier omegaSupplier) {
    CommandSwerveDrivetrain drive = getDrive();
    return Commands.run(() -> drive.driveFieldCentric(xSupplier, ySupplier, omegaSupplier), drive)
        .beforeStarting(Commands.runOnce(() -> claimDriveMode(DriveMode.MANUAL), this));
  }

  /**
   * Sets {@link ControlMode} and claims {@link DriveMode#AUTO_AIM}, then returns the matching shoot
   * command.
   */
  public Command getShootCommand(ControlMode mode, Button shootButton) {
    Command shoot =
        switch (mode) {
          case SCORE -> new HybridScoreCommand(getDrive(), shootButton);
          case PASS -> new HybridPassCommand(getDrive(), shootButton);
          case MANUAL -> new ManualShootCommand(getDrive(), shootButton);
        };
    return shoot.beforeStarting(
        Commands.runOnce(
            () -> {
              setControlMode(mode);
              claimDriveMode(DriveMode.AUTO_AIM);
            },
            this));
  }

  public HeatupCommand getHeatupCommand() {
    return new HeatupCommand();
  }

  public Command getHybridTrenchCommand() {
    return new DriveHybridTrenchCommand(getDrive())
        .beforeStarting(Commands.runOnce(() -> claimDriveMode(DriveMode.HYBRID_TRENCH), this));
  }

  public Command getHybridIntakeCommand(
      ImprovedCommandXboxController controller,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier) {
    return new IntakeHybridCommand(getDrive(), controller, xSupplier, ySupplier, omegaSupplier)
        .beforeStarting(
            Commands.runOnce(() -> claimDriveMode(DriveMode.HYBRID_INTAKE_DRIVE), this));
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

  @Override
  public void periodic() {
    Logger.recordOutput("SuperStructure/DriveMode", driveMode);
    Logger.recordOutput("SuperStructure/ControlMode", controlMode);
    Logger.recordOutput("SuperStructure/ShootPhase", shootPhase);
    Logger.recordOutput("SuperStructure/IntakeMode", intakeMode);
    Logger.recordOutput("SuperStructure/ManualShootVelocityRps", manualShootVelocityRps.get());
    Logger.recordOutput("SuperStructure/ManualHoodDegs", manualHoodDegs.get());
  }
}
