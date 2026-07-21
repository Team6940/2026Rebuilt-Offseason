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

public class RightDoubleSwipeTrench extends SequentialCommandGroup {
  CommandSwerveDrivetrain drive = CommandSwerveDrivetrain.getInstance();
  SuperStructure superStructure = SuperStructure.getInstance();
  ShooterSubsystem shooter = ShooterSubsystem.getInstance();

  public RightDoubleSwipeTrench() {
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
                    drive.generatePPPath("RSt-RInt1OverMid").getStartingHolonomicPose().get());
              } else {
                drive.resetPose(
                    drive
                        .generatePPPath("RSt-RInt1OverMid")
                        .flipPath()
                        .getStartingHolonomicPose()
                        .get());
              }
            }));

    // route
    addCommands(
        superStructure.runOnce(
            () -> superStructure.setShootPhase(SuperStructure.ShootPhase.HEATUP)));
    addCommands(shooter.runOnce(() -> shooter.setVelocityRps(40.)));
    addCommands(
        drive
            .followPPPath("RSt-RInt1OverMid")
            .alongWith(
                Commands.waitSeconds(0.3)
                    .andThen(
                        superStructure.runOnce(
                            () -> superStructure.setIntakeMode(IntakeMode.INTAKE)))));
    addCommands(drive.followPPPath("RInt1OverMid-RTrenchSh1"));
    addCommands(
        superStructure.getShootCommand(ControlMode.SCORE, Button.kAutoButton).withTimeout(2.5));

    addCommands(superStructure.runOnce(() -> superStructure.setIntakeMode(IntakeMode.INTAKE)));
    addCommands(drive.followPPPath("RTrenchSh1-RInt2"));
    addCommands(drive.followPPPath("RInt2-RTrenchSh2"));
    addCommands(
        superStructure.getShootCommand(ControlMode.SCORE, Button.kAutoButton).withTimeout(2.5));

    addCommands(superStructure.runOnce(() -> superStructure.setIntakeMode(IntakeMode.INTAKE)));
    addCommands(drive.followPPPath("RTrenchSh2-REndInt3"));
  }
}
