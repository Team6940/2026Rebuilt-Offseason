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

public class RightDoubleSwipe extends SequentialCommandGroup {
  CommandSwerveDrivetrain drive = CommandSwerveDrivetrain.getInstance();
  SuperStructure superStructure = SuperStructure.getInstance();

  public RightDoubleSwipe() {
    addCommands(
        Commands.defer(
            () -> Commands.waitSeconds(RobotContainer.autoDelaySeconds.get()), Set.of()));
    // init
    addCommands(
        new InstantCommand(
            () -> {
              if (DriverStation.getAlliance().get() == Alliance.Blue) {
                drive.setPose(drive.generatePPPath("RSt-RInt1").getStartingHolonomicPose().get());
              } else {
                drive.setPose(
                    drive.generatePPPath("RSt-RInt1").flipPath().getStartingHolonomicPose().get());
              }
            }));

    // route
    addCommands(superStructure.runOnce(() -> superStructure.setIntakeMode(IntakeMode.INTAKE)));
    addCommands(drive.followPPPath("RSt-RInt1"));

    addCommands(drive.followPPPath("RInt1-RSh1"));
    addCommands(
        new HybridShootCommand(Button.kAutoButton, Button.kAutoButton, Button.kY).withTimeout(3.));

    addCommands(superStructure.runOnce(() -> superStructure.setIntakeMode(IntakeMode.INTAKE)));
    addCommands(drive.followPPPath("RSh1-RInt2"));

    addCommands(drive.followPPPath("RInt2-RSh2"));
    addCommands(
        new HybridShootCommand(Button.kAutoButton, Button.kAutoButton, Button.kY).withTimeout(3.));

    addCommands(superStructure.runOnce(() -> superStructure.setIntakeMode(IntakeMode.INTAKE)));
    addCommands(drive.followPPPath("RSh2-REndInt3"));
  }
}
