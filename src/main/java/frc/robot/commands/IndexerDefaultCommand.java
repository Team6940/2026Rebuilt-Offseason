package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.Indexer.IndexerSubsystem;
import frc.robot.subsystems.SuperStructure;

/** Default intake behavior driven by {@link SuperStructure.IntakeMode}. */
public class IndexerDefaultCommand extends Command {
  private final IndexerSubsystem indexer = IndexerSubsystem.getInstance();
  private final SuperStructure superStructure = SuperStructure.getInstance();

  public IndexerDefaultCommand() {
    addRequirements(indexer);
  }

  @Override
  public void execute() {
    switch (superStructure.getIntakeMode()) {
      case REVERSE -> runReverse();
      default -> runOff();
    }
  }

  private void runOff() {
    indexer.stop();
  }

  private void runReverse() {
    indexer.setVelocities(-20., -20.);
  }

  @Override
  public void end(boolean interrupted) {
    indexer.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
