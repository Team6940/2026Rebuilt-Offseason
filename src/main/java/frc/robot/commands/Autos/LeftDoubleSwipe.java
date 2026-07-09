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

public class LeftDoubleSwipe extends SequentialCommandGroup {
  CommandSwerveDrivetrain drive = CommandSwerveDrivetrain.getInstance();
  ShooterSubsystem shooter = ShooterSubsystem.getInstance();
  SuperStructure superStructure = SuperStructure.getInstance();

  public LeftDoubleSwipe() {
    addCommands(
        Commands.defer(
            () -> Commands.waitSeconds(RobotContainer.autoDelaySeconds.get()), Set.of()));
    addCommands(
        new InstantCommand(
            () -> {
              if (DriverStation.getAlliance().get() == Alliance.Blue) {
                drive.resetPose(drive.generatePPPath("LSt-LInt1").getStartingHolonomicPose().get());
              } else {
                drive.resetPose(
                    drive.generatePPPath("LSt-LInt1").flipPath().getStartingHolonomicPose().get());
              }
            }));

    addCommands(
        superStructure.runOnce(
            () -> superStructure.setShootPhase(SuperStructure.ShootPhase.HEATUP)));
    addCommands(shooter.runOnce(() -> shooter.setVelocityRps(33.3)));
    addCommands(
        drive
            .followPPPath("LSt-LInt1")
            .alongWith(
                Commands.waitSeconds(0.3)
                    .andThen(
                        superStructure.runOnce(
                            () -> superStructure.setIntakeMode(IntakeMode.INTAKE)))));
    addCommands(
        drive
            .followPPPath("LInt1-LSh1")
            .alongWith(
                superStructure.runOnce(() -> superStructure.setIntakeMode(IntakeMode.INTAKE))));
    addCommands(
        superStructure.getShootCommand(ControlMode.SCORE, Button.kAutoButton).withTimeout(2.));

    addCommands(superStructure.runOnce(() -> superStructure.setIntakeMode(IntakeMode.INTAKE)));
    addCommands(drive.followPPPath("LSh1-LInt2"));
    addCommands(superStructure.runOnce(() -> superStructure.setIntakeMode(IntakeMode.MID)));
    addCommands(drive.followPPPath("LInt2-LSh2"));
    addCommands(
        superStructure.getShootCommand(ControlMode.SCORE, Button.kAutoButton).withTimeout(2.));

    addCommands(superStructure.runOnce(() -> superStructure.setIntakeMode(IntakeMode.INTAKE)));
    addCommands(drive.followPPPath("LSh2-LEndInt3"));
  }
}
