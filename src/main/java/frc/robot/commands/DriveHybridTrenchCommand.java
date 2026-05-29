package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Chassis.CommandSwerveDrivetrain;
import frc.robot.subsystems.SuperStructure;
import frc.robot.subsystems.SuperStructure.DriveMode;
import frc.robot.subsystems.SuperStructure.IntakeMode;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

/**
 * Hybrid trench shared-control drive. Higher priority than {@link DriveHybridIntakeCommand}.
 *
 * <p>IntakeMode: unchanged while A only; HYBRID when A and RT together. DriveMode: HYBRID_TRENCH
 * while held; MANUAL on normal end.
 */
public class DriveHybridTrenchCommand extends Command {
  private final CommandSwerveDrivetrain drive;
  private final SuperStructure superStructure = SuperStructure.getInstance();
  private final DoubleSupplier xSupplier;
  private final DoubleSupplier ySupplier;
  private final DoubleSupplier omegaSupplier;
  private final DoubleSupplier maxLinearSpeedSupplier;
  private final DoubleSupplier maxAngularSpeedSupplier;
  private final BooleanSupplier aHeld;
  private final BooleanSupplier rtHeld;

  private IntakeMode intakeModeBeforeTrench = IntakeMode.OFF;

  public DriveHybridTrenchCommand(
      CommandSwerveDrivetrain drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier,
      BooleanSupplier aHeld,
      BooleanSupplier rtHeld) {
    this(
        drive,
        xSupplier,
        ySupplier,
        omegaSupplier,
        aHeld,
        rtHeld,
        drive::getMaxLinearSpeedMetersPerSec,
        drive::getMaxAngularSpeedRadPerSec);
  }

  public DriveHybridTrenchCommand(
      CommandSwerveDrivetrain drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier,
      BooleanSupplier aHeld,
      BooleanSupplier rtHeld,
      DoubleSupplier maxLinearSpeedSupplier,
      DoubleSupplier maxAngularSpeedSupplier) {
    this.drive = drive;
    this.xSupplier = xSupplier;
    this.ySupplier = ySupplier;
    this.omegaSupplier = omegaSupplier;
    this.aHeld = aHeld;
    this.rtHeld = rtHeld;
    this.maxLinearSpeedSupplier = maxLinearSpeedSupplier;
    this.maxAngularSpeedSupplier = maxAngularSpeedSupplier;
    addRequirements(drive);
  }

  @Override
  public void initialize() {
    intakeModeBeforeTrench = superStructure.getIntakeMode();
    drive.resetHybridTrenchState();
    applyTrenchDriveState();
  }

  @Override
  public void execute() {
    applyTrenchDriveState();
    drive.driveHybridTrench(
        xSupplier,
        ySupplier,
        omegaSupplier,
        maxLinearSpeedSupplier.getAsDouble(),
        maxAngularSpeedSupplier.getAsDouble());
  }

  /** A+RT → HYBRID intake; A only → preserve intake mode captured at init. */
  private void applyTrenchDriveState() {
    superStructure.setDriveMode(DriveMode.HYBRID_TRENCH);
    if (aHeld.getAsBoolean() && rtHeld.getAsBoolean()) {
      superStructure.setIntakeMode(IntakeMode.HYBRID);
    } else {
      superStructure.setIntakeMode(intakeModeBeforeTrench);
    }
  }

  @Override
  public void end(boolean interrupted) {
    if (!interrupted) {
      superStructure.setDriveMode(DriveMode.MANUAL);
    }
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
