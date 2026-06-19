package frc.robot.commands.Autos;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.HybridShootCommand;
import frc.robot.subsystems.Chassis.CommandSwerveDrivetrain;
import frc.robot.subsystems.ImprovedCommandXboxController.Button;
import frc.robot.subsystems.SuperStructure;
import frc.robot.subsystems.SuperStructure.IntakeMode;
import java.util.Set;

public class LeftDoubleSwipe extends SequentialCommandGroup {
  CommandSwerveDrivetrain drive = CommandSwerveDrivetrain.getInstance();
  SuperStructure superStructure = SuperStructure.getInstance();

  public LeftDoubleSwipe() {
    addCommands(
        Commands.defer(
            () -> Commands.waitSeconds(RobotContainer.autoDelaySeconds.get()), Set.of()));
    addCommands(
        new InstantCommand(
            () -> {
              if (DriverStation.getAlliance().get() == Alliance.Blue) {
                drive.setPose(drive.generatePPPath("LSt-LInt1").getStartingHolonomicPose().get());
              } else {
                drive.setPose(
                    drive.generatePPPath("LSt-LInt1").flipPath().getStartingHolonomicPose().get());
              }
            }));

    addCommands(superStructure.runOnce(() -> superStructure.setIntakeMode(IntakeMode.INTAKE)));
    addCommands(drive.followPPPath("LSt-LInt1"));

    addCommands(drive.followPPPath("LInt1-LSh1"));
    addCommands(
        new HybridShootCommand(Button.kAutoButton, Button.kAutoButton, Button.kY).withTimeout(3.));

    addCommands(superStructure.runOnce(() -> superStructure.setIntakeMode(IntakeMode.INTAKE)));
    addCommands(drive.followPPPath("LSh1-LInt2"));

    addCommands(drive.followPPPath("LInt2-LSh2"));
    addCommands(
        new HybridShootCommand(Button.kAutoButton, Button.kAutoButton, Button.kY).withTimeout(3.));

    addCommands(superStructure.runOnce(() -> superStructure.setIntakeMode(IntakeMode.INTAKE)));
    addCommands(drive.followPPPath("LSh2-LEndInt3"));
  }
}
