package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class SuperStructure extends SubsystemBase {
  private static SuperStructure instance;

  public static SuperStructure getInstance() {
    if (instance == null) {
      instance = new SuperStructure();
    }
    return instance;
  }

  private SuperStructure() {
    // Private constructor for singleton pattern
  }

  public enum DriveMode {
    HYBRID_TRENCH,
    HYBRID_INTAKE_DRIVE,
    MANUAL
  }

  /** Hub score vs pass lane selection. */
  public enum ControlMode {
    SCORE,
    PASS
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
    REVERSE
  }

  private DriveMode driveModeMode = DriveMode.MANUAL;
  private ControlMode controlMode = ControlMode.SCORE;
  private ShootPhase shootPhase = ShootPhase.OFF;
  private IntakeMode intakeMode = IntakeMode.OFF;

  /** Claim the active drive mode while a hybrid command is running (or MANUAL when released). */
  public void claimDriveMode(DriveMode mode) {
    driveModeMode = mode;
  }

  public void resetAllModes() {
    claimDriveMode(DriveMode.MANUAL);
    setIntakeMode(IntakeMode.OFF);
    setShootPhase(ShootPhase.OFF);
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
    controlMode = controlMode == ControlMode.SCORE ? ControlMode.PASS : ControlMode.SCORE;
  }

  public DriveMode getDriveMode() {
    return driveModeMode;
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

  @Override
  public void periodic() {
    Logger.recordOutput("SuperStructure/DriveMode", driveModeMode);
    Logger.recordOutput("SuperStructure/ControlMode", controlMode);
    Logger.recordOutput("SuperStructure/ShootPhase", shootPhase);
    Logger.recordOutput("SuperStructure/IntakeMode", intakeMode);
  }
}
