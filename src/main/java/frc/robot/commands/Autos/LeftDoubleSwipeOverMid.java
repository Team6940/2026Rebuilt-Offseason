// not very useful

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

public class LeftDoubleSwipeOverMid extends SequentialCommandGroup {
  CommandSwerveDrivetrain drive = CommandSwerveDrivetrain.getInstance();
  SuperStructure superStructure = SuperStructure.getInstance();
  ShooterSubsystem shooter = ShooterSubsystem.getInstance();

  public LeftDoubleSwipeOverMid() {
    addCommands(superStructure.runOnce(() -> superStructure.setIntakeMode(IntakeMode.OFF)));
    addCommands(
        Commands.defer(
            () -> Commands.waitSeconds(RobotContainer.autoDelaySeconds.get()), Set.of()));
    // init
    addCommands(
        new InstantCommand(
            () -> {
              if (DriverStation.getAlliance().get() == Alliance.Blue) {
                drive.resetPose(
                    drive.generatePPPath("LSt-LInt1OverMid").getStartingHolonomicPose().get());
              } else {
                drive.resetPose(
                    drive
                        .generatePPPath("LSt-LInt1OverMid")
                        .flipPath()
                        .getStartingHolonomicPose()
                        .get());
              }
            }));

    // route
    addCommands(
        superStructure.runOnce(
            () -> superStructure.setShootPhase(SuperStructure.ShootPhase.HEATUP)));
    addCommands(shooter.runOnce(() -> shooter.setVelocityRps(33.3)));
    addCommands(
        drive
            .followPPPath("LSt-LInt1OverMid")
            .alongWith(
                Commands.waitSeconds(0.3)
                    .andThen(
                        superStructure.runOnce(
                            () -> superStructure.setIntakeMode(IntakeMode.INTAKE)))));
    addCommands(drive.followPPPath("LInt1OverMid-LSh1"));
    addCommands(
        superStructure.getShootCommand(ControlMode.SCORE, Button.kAutoButton).withTimeout(2.));
    addCommands(shooter.runOnce(() -> shooter.setVelocityRps(33.3)));
    addCommands(superStructure.runOnce(() -> superStructure.setIntakeMode(IntakeMode.INTAKE)));
    addCommands(drive.followPPPath("LSh1-LInt2"));
    addCommands(drive.followPPPath("LInt2-LSh2"));
    addCommands(
        superStructure.getShootCommand(ControlMode.SCORE, Button.kAutoButton).withTimeout(2.));
    addCommands(shooter.runOnce(() -> shooter.setVelocityRps(33.3)));
    addCommands(superStructure.runOnce(() -> superStructure.setIntakeMode(IntakeMode.INTAKE)));
    addCommands(drive.followPPPath("LSh2-LEndInt3"));
  }
}
