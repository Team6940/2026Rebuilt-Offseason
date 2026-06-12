package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Chassis.CommandSwerveDrivetrain;
import frc.robot.subsystems.Shooter.ShooterSubsystem;
import frc.robot.subsystems.SuperStructure;
import frc.robot.subsystems.SuperStructure.ShootPhase;
import frc.robot.util.ProjectileCalculator;

/** Operator RT: spin up shooter to hub distance table speed without hood or drive aim. */
public class HeatupCommand extends Command {
  private final CommandSwerveDrivetrain drive;
  private final ShooterSubsystem shooter = ShooterSubsystem.getInstance();
  private final SuperStructure superStructure = SuperStructure.getInstance();

  public HeatupCommand(CommandSwerveDrivetrain drive) {
    this.drive = drive;
    addRequirements(shooter);
  }

  @Override
  public void initialize() {
    superStructure.setShootPhase(ShootPhase.HEATUP);
  }

  @Override
  public void execute() {
    double dist = drive.getDistanceToTarget(CommandSwerveDrivetrain.getAllianceHubCenter());
    shooter.setVelocityRps(ProjectileCalculator.getShooterTargetVelocity(dist));
  }

  @Override
  public void end(boolean interrupted) {
    shooter.stop();
    if (superStructure.getShootPhase() == ShootPhase.HEATUP) {
      superStructure.setShootPhase(ShootPhase.OFF);
    }
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
