package frc.robot.commands.Drive;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Chassis.CommandSwerveDrivetrain;
import frc.robot.subsystems.SuperStructure;
import frc.robot.subsystems.SuperStructure.DriveMode;
import frc.robot.subsystems.SuperStructure.IntakeMode;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

/**
 * Default drive command selected by {@link SuperStructure#getDriveMode()}. Hybrid trench and
 * hybrid intake behavior live here; {@link SuperStructure#setDriveMode(DriveMode, boolean)}
 * resolves button priority (HybridTrench &gt; HybridIntake &gt; Manual).
 */
public class DefaultDriveCommands {
  private DefaultDriveCommands() {}

  public static Command defaultDrive(
      CommandSwerveDrivetrain drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier,
      BooleanSupplier aHeld,
      BooleanSupplier rtHeld) {
    return new DefaultDriveCommand(drive, xSupplier, ySupplier, omegaSupplier, aHeld, rtHeld);
  }

  private static class DefaultDriveCommand extends Command {
    private final CommandSwerveDrivetrain drive;
    private final SuperStructure superStructure = SuperStructure.getInstance();
    private final DoubleSupplier xSupplier;
    private final DoubleSupplier ySupplier;
    private final DoubleSupplier omegaSupplier;
    private final BooleanSupplier aHeld;
    private final BooleanSupplier rtHeld;

    private DriveMode previousMode = DriveMode.MANUAL;
    private IntakeMode intakeModeBeforeTrench = IntakeMode.OFF;

    private DefaultDriveCommand(
        CommandSwerveDrivetrain drive,
        DoubleSupplier xSupplier,
        DoubleSupplier ySupplier,
        DoubleSupplier omegaSupplier,
        BooleanSupplier aHeld,
        BooleanSupplier rtHeld) {
      this.drive = drive;
      this.xSupplier = xSupplier;
      this.ySupplier = ySupplier;
      this.omegaSupplier = omegaSupplier;
      this.aHeld = aHeld;
      this.rtHeld = rtHeld;
      addRequirements(drive);
    }

    @Override
    public void initialize() {
      previousMode = superStructure.getDriveMode();
    }

    @Override
    public void execute() {
      DriveMode mode = superStructure.getDriveMode();
      onDriveModeChanged(previousMode, mode);
      previousMode = mode;

      switch (mode) {
        case HYBRID_TRENCH -> executeHybridTrench();
        case HYBRID_INTAKE_DRIVE -> executeHybridIntake();
        case MANUAL -> executeManual();
      }
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
      return false;
    }

    private void onDriveModeChanged(DriveMode from, DriveMode to) {
      if (to == DriveMode.HYBRID_TRENCH && from != DriveMode.HYBRID_TRENCH) {
        intakeModeBeforeTrench = superStructure.getIntakeMode();
        drive.resetHybridTrenchState();
      }
      if (to == DriveMode.HYBRID_INTAKE_DRIVE && from != DriveMode.HYBRID_INTAKE_DRIVE) {
        superStructure.setIntakeMode(IntakeMode.HYBRID);
      }
      if (from == DriveMode.HYBRID_INTAKE_DRIVE && to == DriveMode.MANUAL) {
        superStructure.setIntakeMode(IntakeMode.INTAKE);
      }
    }

    private void executeManual() {
      drive.driveFieldCentric(xSupplier, ySupplier, omegaSupplier);
    }

    private void executeHybridIntake() {
      drive.driveHybridIntake(
          xSupplier,
          ySupplier,
          omegaSupplier,
          drive.getMaxLinearSpeedMetersPerSec(),
          drive.getMaxAngularSpeedRadPerSec());
    }

    private void executeHybridTrench() {
      applyTrenchIntakeMode(aHeld.getAsBoolean(), rtHeld.getAsBoolean());
      drive.driveHybridTrench(
          xSupplier,
          ySupplier,
          omegaSupplier,
          drive.getMaxLinearSpeedMetersPerSec(),
          drive.getMaxAngularSpeedRadPerSec());
    }

    /** A+RT → HYBRID intake; A only → preserve intake mode captured at trench entry. */
    private void applyTrenchIntakeMode(boolean aHeld, boolean rtHeld) {
      if (aHeld && rtHeld) {
        superStructure.setIntakeMode(IntakeMode.HYBRID);
      } else {
        superStructure.setIntakeMode(intakeModeBeforeTrench);
      }
    }
  }
}
