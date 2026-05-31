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
  private IntakeMode intakeMode = IntakeMode.HYBRID;

  private boolean hybridTrenchRequested = false;
  private boolean hybridIntakeRequested = false;

  /** Priority follows {@link DriveMode} enum order: HYBRID_TRENCH &gt; HYBRID_INTAKE_DRIVE &gt; MANUAL. */
  public void setDriveMode(DriveMode mode, boolean requested) {
    switch (mode) {
      case HYBRID_TRENCH -> hybridTrenchRequested = requested;
      case HYBRID_INTAKE_DRIVE -> hybridIntakeRequested = requested;
      case MANUAL -> {
        if (requested) {
          hybridTrenchRequested = false;
          hybridIntakeRequested = false;
        }
      }
    }
    resolveDriveMode();
  }

  private void resolveDriveMode() {
    for (DriveMode mode : DriveMode.values()) {
      switch (mode) {
        case HYBRID_TRENCH -> {
          if (hybridTrenchRequested) {
            driveModeMode = DriveMode.HYBRID_TRENCH;
            return;
          }
        }
        case HYBRID_INTAKE_DRIVE -> {
          if (hybridIntakeRequested) {
            driveModeMode = DriveMode.HYBRID_INTAKE_DRIVE;
            return;
          }
        }
        case MANUAL -> driveModeMode = DriveMode.MANUAL;
      }
    }
  }

  public void resetDriveMode() {
    hybridTrenchRequested = false;
    hybridIntakeRequested = false;
    driveModeMode = DriveMode.MANUAL;
  }

  public void resetIntakeMode() {
    intakeMode = IntakeMode.OFF;
  }

  public void resetAllModes() {
    resetDriveMode();
    resetIntakeMode();
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
