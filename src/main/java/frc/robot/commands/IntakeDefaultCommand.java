package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.Intake.IntakeSubsystem;
import frc.robot.subsystems.SuperStructure;

/** Default intake behavior driven by {@link SuperStructure.IntakeMode}. */
public class IntakeDefaultCommand extends Command {
  private final IntakeSubsystem intake = IntakeSubsystem.getInstance();
  private final SuperStructure superStructure = SuperStructure.getInstance();

  public IntakeDefaultCommand() {
    addRequirements(intake);
  }

  @Override
  public void execute() {
    switch (superStructure.getIntakeMode()) {
      case INTAKE -> runIntake();
      case HYBRID -> runHybrid();
      case RETRACTED -> runRetracted();
      case OFF -> runOff();
      case REVERSE -> runReverse();
    }
  }

  private void runIntake() {
    intake.setRollerRps(IntakeConstants.IntakingRps);
    intake.setRackPosition(IntakeConstants.RackExtendedRotations);
  }

  private void runHybrid() {
    intake.setRollerRps(IntakeConstants.IntakingRps);
    intake.setRackPosition(IntakeConstants.RackExtendedRotations);
  }

  private void runRetracted() {
    intake.stopRollers();
    intake.setRackPosition(IntakeConstants.RackRetractedRotations);
  }

  private void runOff() {
    intake.stopRollers();
    intake.setRackPosition(IntakeConstants.RackIdleRotations);
  }

  private void runReverse() {
    intake.setRollerRps(-IntakeConstants.IntakingRps);
    intake.setRackPosition(IntakeConstants.RackExtendedRotations);
  }

  @Override
  public void end(boolean interrupted) {
    intake.stopRollers();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
