// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import com.pathplanner.lib.commands.FollowPathCommand;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.DriveHybridIntakeCommand;
import frc.robot.commands.DriveHybridTrenchCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Chassis.CommandSwerveDrivetrain;
import frc.robot.subsystems.ImprovedCommandXboxController;
import frc.robot.subsystems.SuperStructure;
import frc.robot.subsystems.Vision.VisionSubsystem;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  public static final String limelightLeft = "limelight-l";
  public static final String limelightRight = "limelight";
  public static final String photonCameraName = "photonvision";

  private final CommandSwerveDrivetrain drive;
  private final VisionSubsystem vision;
  private final SuperStructure superStructure = SuperStructure.getInstance();

  // Controller
  public static final ImprovedCommandXboxController driverController =
      new ImprovedCommandXboxController(0);
  public static final ImprovedCommandXboxController operatorController =
      new ImprovedCommandXboxController(1);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    drive = TunerConstants.createDrivetrain();
    vision = new VisionSubsystem(drive);

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices");

    // Set up SysId routines
    // autoChooser.addOption(
    //     "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    // autoChooser.addOption(
    //     "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    // autoChooser.addOption(
    //     "Drive SysId (Quasistatic Forward)",
    //     drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    // autoChooser.addOption(
    //     "Drive SysId (Quasistatic Reverse)",
    //     drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    // autoChooser.addOption(
    //     "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    // autoChooser.addOption(
    //     "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    configureButtonBindings();
    // testBindings();

    CommandScheduler.getInstance().schedule(FollowPathCommand.warmupCommand());
    CommandScheduler.getInstance().schedule(
        Commands.runOnce(drive::warmupHybridTrenchControlLoop, drive).ignoringDisable(true));
  }

  private void testBindings() {}

  /** ***** THE CONTROL LOGIC IS SUCH ****** */
  private void configureButtonBindings() {
    drive.setDefaultCommand(
        Commands.run(
            () ->
                drive.driveFieldCentric(
                    () -> -driverController.getLeftY(),
                    () -> -driverController.getLeftX(),
                    () -> -driverController.getRightX()),
            drive));

    // A (HybridTrench) interrupts RT (HybridIntake) via end(true). IntakeMode rules in commands.
    driverController
        .rightTrigger()
        .whileTrue(
            new DriveHybridIntakeCommand(
                drive,
                () -> -driverController.getLeftY(),
                () -> -driverController.getLeftX(),
                () -> -driverController.getRightX()));

    driverController
        .a()
        .whileTrue(
            new DriveHybridTrenchCommand(
                drive,
                () -> -driverController.getLeftY(),
                () -> -driverController.getLeftX(),
                () -> -driverController.getRightX(),
                driverController.a()::getAsBoolean,
                driverController.rightTrigger()::getAsBoolean));

    operatorController
        .povDown()
        .onTrue(Commands.runOnce(superStructure::resetAllModes));

    driverController.x().onTrue(Commands.runOnce(drive::stopWithX, drive));
    driverController
        .b()
        .onTrue(
            Commands.runOnce(
                    () -> drive.setPose(new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
                    drive)
                .ignoringDisable(true));

    driverController
        .leftTrigger()
        .whileTrue(
            drive.run(
                () ->
                    drive.driveFieldCentricWithMaxSpeed(
                        () -> -driverController.getLeftY(),
                        () -> -driverController.getLeftX(),
                        () -> -driverController.getRightX(),
                        1.6,
                        5.4)));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  public void resetSimulationField() {
    if (Constants.currentMode != Constants.Mode.SIM) return;
    drive.setPose(new Pose2d(0.7, 0.7, new Rotation2d()));
  }

  public void updateSimulation() {
    if (Constants.currentMode != Constants.Mode.SIM) return;
    Logger.recordOutput("FieldSimulation/RobotPosition", drive.getPose());
  }
}
