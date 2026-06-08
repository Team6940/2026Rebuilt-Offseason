package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.Chassis.CommandSwerveDrivetrain;
import frc.robot.subsystems.ImprovedCommandXboxController;
import frc.robot.subsystems.SuperStructure;
import frc.robot.subsystems.SuperStructure.IntakeMode;
import java.util.function.DoubleSupplier;

/**
 * LT while held: {@link CommandSwerveDrivetrain#driveHybridIntake}. Short press → {@link
 * IntakeMode#INTAKE}; hold past threshold → {@link IntakeMode#HYBRID}.
 */
public class IntakeHybridCommand extends Command {
  private final CommandSwerveDrivetrain drive;
  private final ImprovedCommandXboxController controller;
  private final SuperStructure superStructure = SuperStructure.getInstance();
  private final DoubleSupplier xSupplier;
  private final DoubleSupplier ySupplier;
  private final DoubleSupplier omegaSupplier;

  private double ltPressStartSec = -1.0;
  private boolean ltWasPressed = false;

  public IntakeHybridCommand(
      CommandSwerveDrivetrain drive,
      ImprovedCommandXboxController controller,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier) {
    this.drive = drive;
    this.controller = controller;
    this.xSupplier = xSupplier;
    this.ySupplier = ySupplier;
    this.omegaSupplier = omegaSupplier;
    addRequirements(drive);
  }

  @Override
  public void execute() {
    boolean ltPressed = controller.getLeftTrigger();
    double now = Timer.getFPGATimestamp();

    if (ltPressed && !ltWasPressed) {
      ltPressStartSec = now;
      superStructure.setIntakeMode(IntakeMode.INTAKE);
    } else if (ltPressed
        && ltPressStartSec >= 0.0
        && now - ltPressStartSec >= IntakeConstants.LtHoldThresholdSec) {
      superStructure.setIntakeMode(IntakeMode.HYBRID);
    } else if (!ltPressed) {
      ltPressStartSec = -1.0;
    }
    ltWasPressed = ltPressed;

    if (ltPressed) {
      drive.driveHybridIntake(xSupplier, ySupplier, omegaSupplier, 1.6, 5.4);
    }
  }

  @Override
  public void end(boolean interrupted) {
    superStructure.setIntakeMode(IntakeMode.INTAKE);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
