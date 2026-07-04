package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter.ShooterSubsystem;
import frc.robot.subsystems.SuperStructure;
import frc.robot.subsystems.SuperStructure.ShootPhase;

/** Operator RT: spin up shooter to hub distance table speed without hood or drive aim. */
public class HeatupCommand extends Command {
  private final ShooterSubsystem shooter = ShooterSubsystem.getInstance();
  private final SuperStructure superStructure = SuperStructure.getInstance();

  public HeatupCommand() {
    addRequirements(shooter);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    if (superStructure.getShootPhase() == ShootPhase.OFF) {
      shooter.stop();
    } else if (superStructure.getShootPhase() == ShootPhase.HEATUP) {
      shooter.setVelocityRps(33.);
    }
  }

  @Override
  public void end(boolean interrupted) {
    shooter.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
