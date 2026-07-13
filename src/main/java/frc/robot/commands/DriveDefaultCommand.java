package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Chassis.CommandSwerveDrivetrain;
import frc.robot.subsystems.SuperStructure;
import frc.robot.subsystems.SuperStructure.DriveMode;
import java.util.function.DoubleSupplier;

public class DriveDefaultCommand extends Command {
  private final CommandSwerveDrivetrain drive;
  private final SuperStructure superStructure = SuperStructure.getInstance();
  private final DoubleSupplier xSupplier;
  private final DoubleSupplier ySupplier;
  private final DoubleSupplier omegaSupplier;
  private final DoubleSupplier maxLinearSpeedSupplier;
  private final DoubleSupplier maxAngularSpeedSupplier;

  public DriveDefaultCommand(
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

  public DriveDefaultCommand(
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
    superStructure.claimDriveMode(DriveMode.MANUAL);
  }

  @Override
  public void execute() {
    superStructure.claimDriveMode(DriveMode.MANUAL);
    if (!superStructure.getFieldCentricEnbaled()) {
      drive.driveRobotCentric(xSupplier, ySupplier, omegaSupplier);
    } else {
      drive.driveFieldCentric(xSupplier, ySupplier, omegaSupplier);
    }
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
