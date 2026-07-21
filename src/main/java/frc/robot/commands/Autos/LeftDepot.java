package frc.robot.commands.Autos;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Chassis.CommandSwerveDrivetrain;
import frc.robot.subsystems.ImprovedCommandXboxController.Button;
import frc.robot.subsystems.Shooter.ShooterSubsystem;
import frc.robot.subsystems.SuperStructure;
import frc.robot.subsystems.SuperStructure.ControlMode;
import frc.robot.subsystems.SuperStructure.IntakeMode;
import java.util.Set;

public class LeftDepot extends SequentialCommandGroup {
  CommandSwerveDrivetrain drive = CommandSwerveDrivetrain.getInstance();
  ShooterSubsystem shooter = ShooterSubsystem.getInstance();
  SuperStructure superStructure = SuperStructure.getInstance();

  public LeftDepot() {
    addCommands(superStructure.runOnce(() -> superStructure.setIntakeMode(IntakeMode.OFF)));
    addCommands(
        Commands.defer(
            () -> Commands.waitSeconds(RobotContainer.autoDelaySeconds.get()), Set.of()));
    addCommands(
        new InstantCommand(
            () -> {
              if (DriverStation.getAlliance().get() == Alliance.Blue) {
                drive.resetPose(
                    drive.generatePPPath("LSt2-LDepot").getStartingHolonomicPose().get());
              } else {
                drive.resetPose(
                    drive
                        .generatePPPath("LSt2-LDepot")
                        .flipPath()
                        .getStartingHolonomicPose()
                        .get());
              }
            }));

    addCommands(superStructure.runOnce(() -> superStructure.setIntakeMode(IntakeMode.INTAKE)));
    addCommands(
        superStructure.runOnce(
            () -> superStructure.setShootPhase(SuperStructure.ShootPhase.HEATUP)));
    addCommands(shooter.runOnce(() -> shooter.setVelocityRps(33.3)));
    addCommands(drive.followPPPath("LSt2-LDepot"));

    addCommands(drive.followPPPath("LDepot-LSh3"));
    addCommands(
        superStructure.getShootCommand(ControlMode.SCORE, Button.kAutoButton).withTimeout(2.5));

    addCommands(superStructure.runOnce(() -> superStructure.setIntakeMode(IntakeMode.INTAKE)));
  }
}
