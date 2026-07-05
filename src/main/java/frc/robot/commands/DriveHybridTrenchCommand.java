package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Chassis.CommandSwerveDrivetrain;
import frc.robot.subsystems.ImprovedCommandXboxController;
import frc.robot.subsystems.SuperStructure;
import frc.robot.subsystems.SuperStructure.DriveMode;
import frc.robot.subsystems.SuperStructure.IntakeMode;

/**
 * Hybrid trench shared-control drive while driver holds A (see {@link frc.robot.RobotContainer}).
 *
 * <p>IntakeMode: unchanged while A only; HYBRID when A and RT together. DriveMode: HYBRID_TRENCH
 * while held; MANUAL on normal end.
 */
public class DriveHybridTrenchCommand extends Command {
  private final CommandSwerveDrivetrain drive;
  private final ImprovedCommandXboxController driverController = RobotContainer.driverController;
  private final SuperStructure superStructure = SuperStructure.getInstance();

  private IntakeMode intakeModeBeforeTrench = IntakeMode.OFF;

  public DriveHybridTrenchCommand(CommandSwerveDrivetrain drive) {
    this.drive = drive;
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
        () -> -driverController.getLeftY(),
        () -> -driverController.getLeftX(),
        () -> -driverController.getRightX(),
        drive.getMaxLinearSpeedMetersPerSec() * 0.6,
        drive.getMaxAngularSpeedRadPerSec() * 0.6);
  }

  /** RT to HYBRID intake; otherwise preserve intake mode captured at init. */
  private void applyTrenchDriveState() {
    superStructure.claimDriveMode(DriveMode.HYBRID_TRENCH);
    if (driverController.getRightTrigger()) {
      superStructure.setIntakeMode(IntakeMode.HYBRID);
    } else {
      superStructure.setIntakeMode(intakeModeBeforeTrench);
    }
  }

  @Override
  public void end(boolean interrupted) {
    if (!interrupted) {
      superStructure.claimDriveMode(DriveMode.MANUAL);
    }
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
