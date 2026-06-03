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

  public enum ShootMode {
    SCORE,
    PASS
  }

  public enum IntakeMode {
    INTAKE,
    HYBRID,
    OFF,
    REVERSE
  }

  private DriveMode driveModeMode = DriveMode.MANUAL;
  private ShootMode shootMode = ShootMode.SCORE;
  private IntakeMode intakeMode = IntakeMode.OFF;

  /** Claim the active drive mode while a hybrid command is running (or MANUAL when released). */
  public void claimDriveMode(DriveMode mode) {
    driveModeMode = mode;
  }

  public void resetAllModes() {
    claimDriveMode(DriveMode.MANUAL);
    setIntakeMode(IntakeMode.OFF);
  }

  public void setShootMode(ShootMode mode) {
    shootMode = mode;
  }

  public void setIntakeMode(IntakeMode mode) {
    intakeMode = mode;
  }

  public void toggleShootMode() {
    shootMode = shootMode == ShootMode.SCORE ? ShootMode.PASS : ShootMode.SCORE;
  }

  public void toggleIntakeMode() {
    IntakeMode newMode = intakeMode == IntakeMode.INTAKE ? IntakeMode.OFF : IntakeMode.INTAKE;
    setIntakeMode(newMode);
  }

  public DriveMode getDriveMode() {
    return driveModeMode;
  }

  public ShootMode getShootMode() {
    return shootMode;
  }

  public IntakeMode getIntakeMode() {
    return intakeMode;
  }

  @Override
  public void periodic() {
    Logger.recordOutput("SuperStructure/DriveMode", driveModeMode);
    Logger.recordOutput("SuperStructure/ShootMode", shootMode);
    Logger.recordOutput("SuperStructure/IntakeMode", intakeMode);
  }
}
