package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Chassis.CommandSwerveDrivetrain;
import frc.robot.subsystems.SuperStructure;
import frc.robot.subsystems.SuperStructure.DriveMode;
import java.util.function.DoubleSupplier;

/**
 * Hybrid intake drive: heading tracks travel. Lower priority than {@link DriveHybridTrenchCommand}.
 *
 * <p>While held: DriveMode=HYBRID_INTAKE_DRIVE. IntakeMode is set by LT/LB/POV, not this command.
 */
public class DriveHybridIntakeCommand extends Command {
  private final CommandSwerveDrivetrain drive;
  private final SuperStructure superStructure = SuperStructure.getInstance();
  private final DoubleSupplier xSupplier;
  private final DoubleSupplier ySupplier;
  private final DoubleSupplier omegaSupplier;
  private final DoubleSupplier maxLinearSpeedSupplier;
  private final DoubleSupplier maxAngularSpeedSupplier;

  public DriveHybridIntakeCommand(
      CommandSwerveDrivetrain drive,
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
      CommandSwerveDrivetrain drive,
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
    superStructure.claimDriveMode(DriveMode.HYBRID_INTAKE_DRIVE);
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
    superStructure.claimDriveMode(DriveMode.MANUAL);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
