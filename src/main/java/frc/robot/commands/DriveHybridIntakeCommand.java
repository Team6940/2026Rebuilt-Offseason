package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drive.Drive;
import frc.robot.subsystems.SuperStructure;
import frc.robot.subsystems.SuperStructure.DriveMode;
import frc.robot.subsystems.SuperStructure.IntakeMode;
import java.util.function.DoubleSupplier;

/**
 * Hybrid intake drive: heading tracks travel. Lower priority than {@link DriveHybridTrenchCommand}.
 *
 * <p>While held: IntakeMode=HYBRID, DriveMode=HYBRID_INTAKE_DRIVE. On normal end: DriveMode=MANUAL,
 * IntakeMode=INTAKE.
 */
public class DriveHybridIntakeCommand extends Command {
  private final Drive drive;
  private final SuperStructure superStructure = SuperStructure.getInstance();
  private final DoubleSupplier xSupplier;
  private final DoubleSupplier ySupplier;
  private final DoubleSupplier omegaSupplier;
  private final DoubleSupplier maxLinearSpeedSupplier;
  private final DoubleSupplier maxAngularSpeedSupplier;

  public DriveHybridIntakeCommand(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier) {
    this(
        drive,
        xSupplier,
        ySupplier,
        omegaSupplier,
        drive::getMaxLinearSpeedMetersPerSec,
        drive::getMaxAngularSpeedRadPerSec);
  }

  public DriveHybridIntakeCommand(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier,
      DoubleSupplier maxLinearSpeedSupplier,
      DoubleSupplier maxAngularSpeedSupplier) {
    this.drive = drive;
    this.xSupplier = xSupplier;
    this.ySupplier = ySupplier;
    this.omegaSupplier = omegaSupplier;
    this.maxLinearSpeedSupplier = maxLinearSpeedSupplier;
    this.maxAngularSpeedSupplier = maxAngularSpeedSupplier;
    addRequirements(drive);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    drive.driveHybridIntake(
        xSupplier,
        ySupplier,
        omegaSupplier,
        maxLinearSpeedSupplier.getAsDouble(),
        maxAngularSpeedSupplier.getAsDouble());
  }

  @Override
  public void end(boolean interrupted) {
    if (interrupted) {
      return;
    }
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
