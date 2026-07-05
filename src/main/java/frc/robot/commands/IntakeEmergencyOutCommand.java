package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.Intake.IntakeSubsystem;

/** Operator LT: lock rack fully extended and run intake rollers at emergency-out speed. */
public class IntakeEmergencyOutCommand extends Command {
  private final IntakeSubsystem intake = IntakeSubsystem.getInstance();

  public IntakeEmergencyOutCommand() {
    addRequirements(intake);
  }

  @Override
  public void execute() {
    intake.setRollerRps(IntakeConstants.IntakeEmergencyOutRps);
    intake.setRackPosition(IntakeConstants.RackMaxRotations);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
