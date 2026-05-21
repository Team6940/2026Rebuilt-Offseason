package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drive.Drive;
import frc.robot.subsystems.SuperStructure;
import java.util.function.DoubleSupplier;

/**
 * Shared-control holonomic drive: driver retains authority; trench assist only stabilizes
 * translation. Schedule with {@code whileTrue} to override the drive default command while held.
 */
public class DriveAutoTrenchCommand extends Command {
  private final Drive drive;
  private final DoubleSupplier xSupplier;
  private final DoubleSupplier ySupplier;
  private final DoubleSupplier omegaSupplier;
  private final DoubleSupplier maxLinearSpeedSupplier;
  private final DoubleSupplier maxAngularSpeedSupplier;

  public DriveAutoTrenchCommand(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier) {
    this(drive, xSupplier, ySupplier, omegaSupplier, drive::getMaxLinearSpeedMetersPerSec, drive::getMaxAngularSpeedRadPerSec);
  }

  public DriveAutoTrenchCommand(
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
  public void execute() {
    drive.driveAutoTrench(
        xSupplier,
        ySupplier,
        omegaSupplier,
        maxLinearSpeedSupplier.getAsDouble(),
        maxAngularSpeedSupplier.getAsDouble());
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
