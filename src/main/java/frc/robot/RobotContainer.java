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

import static edu.wpi.first.units.Units.MetersPerSecond;

import com.pathplanner.lib.commands.FollowPathCommand;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Constants.Ports.LED;
import frc.robot.commands.Autos.LeftDepotSingleSwipe;
import frc.robot.commands.Autos.LeftDoubleSwipe;
import frc.robot.commands.Autos.RightDoubleSwipe;
import frc.robot.commands.DriveCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.simulation.FieldSimulation;
import frc.robot.subsystems.Chassis.CommandSwerveDrivetrain;
import frc.robot.subsystems.Hood.HoodSubsystem;
import frc.robot.subsystems.ImprovedCommandXboxController;
import frc.robot.subsystems.ImprovedCommandXboxController.Button;
import frc.robot.subsystems.Indexer.IndexerSubsystem;
import frc.robot.subsystems.Intake.IntakeSubsystem;
import frc.robot.subsystems.Shooter.ShooterSubsystem;
import frc.robot.subsystems.SuperStructure;
import frc.robot.subsystems.SuperStructure.ControlMode;
import frc.robot.subsystems.SuperStructure.IntakeMode;
import frc.robot.subsystems.SuperStructure.ShootPhase;
import frc.robot.subsystems.Vision.VisionSubsystem;
import frc.robot.subsystems.leds.LEDController;
import java.util.Set;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  public static final String limelightLeft = "limelight-l";
  public static final String limelightRight = "limelight-r";
  public static final String photonCameraBack = "PhotonBack";
  public static final String photonCameraFront = "PhotonFront";

  // Create all Subsystems
  private final CommandSwerveDrivetrain drive;
  private final VisionSubsystem vision;
  private final IntakeSubsystem intake = IntakeSubsystem.getInstance();
  // private final LEDController leds = LEDController.getInstance();
  private final HoodSubsystem hood = HoodSubsystem.getInstance();
  private final IndexerSubsystem indexer = IndexerSubsystem.getInstance();
  private final ShooterSubsystem shooter = ShooterSubsystem.getInstance();
  private final SuperStructure superStructure = SuperStructure.getInstance();
  private Telemetry logger = new Telemetry(TunerConstants.kSpeedAt12Volts.in(MetersPerSecond));
  // Controller
  public static final ImprovedCommandXboxController driverController =
      new ImprovedCommandXboxController(0);
  public static final ImprovedCommandXboxController operatorController =
      new ImprovedCommandXboxController(1);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;
  public static final LoggedNetworkNumber autoDelaySeconds =
      new LoggedNetworkNumber("SmartDashboard/Auto Delay", 0.0);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    drive = TunerConstants.createDrivetrain();

    if (Constants.currentMode == Constants.Mode.SIM) {
      FieldSimulation.initialize(drive, new Pose2d(0.7, 0.7, new Rotation2d()));
    }

    vision = new VisionSubsystem(drive);

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices");
    autoChooser.addOption("LeftDoubleSwipe", new LeftDoubleSwipe());
    autoChooser.addOption("RightDoubleSwipe", new RightDoubleSwipe());
    autoChooser.addOption("LeftDepotSingleSwipe", new LeftDepotSingleSwipe());

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

    intake.setDefaultCommand(superStructure.getIntakeDefaultCommand());
    shooter.setDefaultCommand(superStructure.getHeatupCommand());
    // leds.setDefaultCommand(superStructure.getLEDDefaultCommand());

    configureButtonBindings();
    // testBindings();
    drive.registerTelemetry(logger::telemeterize);
    CommandScheduler.getInstance().schedule(FollowPathCommand.warmupCommand());
    CommandScheduler.getInstance()
        .schedule(
            Commands.runOnce(drive::warmupHybridTrenchControlLoop, drive).ignoringDisable(true));
  }

  private void testBindings() {
    drive.setDefaultCommand(
        superStructure.getFieldCentricDriveCommand(
            () -> -driverController.getLeftY(),
            () -> -driverController.getLeftX(),
            () -> -driverController.getRightX()));
    driverController
        .b()
        .onTrue(Commands.runOnce(superStructure::resetRobotHeading, drive).ignoringDisable(true));
  }

  /** ***** THE CONTROL LOGIC IS SUCH ****** */
  private void configureButtonBindings() {
    // Drive priority (highest wins): AutoAim (Score/Pass/Manual) > HybridTrench > HybridIntake >
    // Manual
    Trigger hybridScore = driverController.rightBumper().or(driverController.rightTrigger());
    Trigger hybridPass = driverController.y().and(driverController.rightBumper().negate());
    Trigger hybridManual = operatorController.povLeft();
    Trigger hybridTrenchDrive =
        driverController
            .a()
            .and(hybridScore.negate())
            .and(hybridPass.negate())
            .and(hybridManual.negate());
    Trigger hybridIntakeDrive =
        driverController
            .leftTrigger()
            .and(hybridScore.negate())
            .and(hybridPass.negate())
            .and(hybridManual.negate())
            .and(driverController.a().negate());

    drive.setDefaultCommand(
        superStructure.getFieldCentricDriveCommand(
            () -> -driverController.getLeftY(),
            () -> -driverController.getLeftX(),
            () -> -driverController.getRightX()));

    hybridScore.whileTrue(superStructure.getShootCommand(ControlMode.SCORE, Button.kRightTrigger));
    hybridPass.whileTrue(superStructure.getShootCommand(ControlMode.PASS, Button.kRightTrigger));
    hybridManual.toggleOnTrue(
        superStructure.getShootCommand(ControlMode.MANUAL, Button.kRightTrigger));

    operatorController.leftTrigger().onTrue(superStructure.getIntakeEmergencyOutCommand());

    operatorController
        .rightTrigger()
        .onTrue(Commands.runOnce(() -> superStructure.setShootPhase(ShootPhase.HEATUP)));
    operatorController
        .rightBumper()
        .onTrue(Commands.runOnce(() -> superStructure.setShootPhase(ShootPhase.OFF)));

    hybridTrenchDrive.whileTrue(superStructure.getHybridTrenchCommand());

    operatorController
        .povDown()
        .onTrue(Commands.runOnce(superStructure::resetAllModes, superStructure));

    driverController.x().onTrue(Commands.runOnce(superStructure::stopDriveWithX, drive));
    driverController
        .b()
        .onTrue(Commands.runOnce(superStructure::resetRobotHeading, drive).ignoringDisable(true));

    hybridIntakeDrive.whileTrue(
        superStructure.getHybridIntakeCommand(
            driverController,
            () -> -driverController.getLeftY(),
            () -> -driverController.getLeftX(),
            () -> -driverController.getRightX()));

    driverController
        .leftBumper()
        .onTrue(Commands.runOnce(superStructure::cycleIntakeRetractState, superStructure));

    driverController
        .povUp()
        .onTrue(
            Commands.runOnce(
                () -> superStructure.setIntakeMode(IntakeMode.REVERSE), superStructure));
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
    if (Constants.currentMode == Constants.Mode.SIM) {
      Pose2d pose = new Pose2d(0.7, 0.7, new Rotation2d());
      drive.setPose(pose);
      FieldSimulation.getInstance().resetField(pose);
    }
  }

  public void updateSimulation() {
    if (Constants.currentMode == Constants.Mode.SIM) {
      FieldSimulation.getInstance().simulationPeriodic();
    }
  }
}
