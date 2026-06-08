package frc.robot.simulation;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants;
import frc.robot.Constants.FieldSimulationConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Chassis.CommandSwerveDrivetrain;
import java.util.List;
import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.Arena2026Rebuilt;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnField;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnFly;
import org.littletonrobotics.junction.Logger;

/**
 * Maple-sim field layer: drivetrain collision body, OverTheBumper {@link IntakeSimulation}
 * (hopper), and full-width rear dumper {@link RebuiltFuelOnFly}. See <a
 * href="https://shenzhen-robotics-alliance.github.io/maple-sim/rebuilt/">Rebuilt Simulation</a>.
 */
public final class FieldSimulation {
  private static final double WHEEL_COF = 2.255;
  private static final double ROBOT_MASS_KG = 74.088;

  private static FieldSimulation instance;

  private final CommandSwerveDrivetrain drive;
  private final SwerveDriveSimulation driveSimulation;
  private final IntakeSimulation intakeSimulation;

  private FieldSimulation(CommandSwerveDrivetrain drive, Pose2d initialPose) {
    this.drive = drive;

    SimulatedArena.overrideInstance(new Arena2026Rebuilt(false));
    driveSimulation = new SwerveDriveSimulation(createMapleSimConfig(), initialPose);
    SimulatedArena.getInstance().addDriveTrainSimulation(driveSimulation);

    intakeSimulation =
        IntakeSimulation.OverTheBumperIntake(
            FieldSimulationConstants.FUEL_TYPE,
            driveSimulation,
            Meters.of(FieldSimulationConstants.OverTheBumperIntakeWidthMeters),
            Meters.of(FieldSimulationConstants.OverTheBumperIntakeExtensionMeters),
            IntakeSimulation.IntakeSide.FRONT,
            FieldSimulationConstants.HopperCapacity);

    spawnDefaultFuel();
  }

  public static void initialize(CommandSwerveDrivetrain drive, Pose2d initialPose) {
    if (Constants.currentMode == Constants.Mode.SIM) {
      if (instance == null) {
        instance = new FieldSimulation(drive, initialPose);
      }
    }
  }

  public static FieldSimulation getInstance() {
    return instance;
  }

  public SwerveDriveSimulation getDriveSimulation() {
    return driveSimulation;
  }

  public IntakeSimulation getIntakeSimulation() {
    return intakeSimulation;
  }

  public void setIntakeRunning(boolean running) {
    if (running) {
      intakeSimulation.startIntake();
    } else {
      intakeSimulation.stopIntake();
    }
  }

  public int getFuelInIntakeCount() {
    return intakeSimulation.getGamePiecesAmount();
  }

  /** Sync maple-sim robot body with CTRE swerve odometry (kinematic coupling). */
  public void syncDriveState() {
    Pose2d pose = drive.getPose();
    ChassisSpeeds fieldSpeeds =
        ChassisSpeeds.fromRobotRelativeSpeeds(drive.getChassisSpeeds(), pose.getRotation());
    driveSimulation.setSimulationWorldPose(pose);
    driveSimulation.setRobotSpeeds(fieldSpeeds);
  }

  public void simulationPeriodic() {
    syncDriveState();
    SimulatedArena.getInstance().simulationPeriodic();
    logFieldTelemetry();
  }

  public void resetField(Pose2d robotPose) {
    SimulatedArena.getInstance().resetFieldForAuto();
    driveSimulation.setSimulationWorldPose(robotPose);
    spawnDefaultFuel();
  }

  /**
   * Dumps a full-width volley (3 parallel FUEL) from the hopper when enough pieces are available.
   *
   * @return true if at least one projectile was added to the arena
   */
  public boolean tryLaunchFuel(double hoodDegs, double shooterRps) {
    Pose2d robotPose = driveSimulation.getSimulatedDriveTrainPose();
    Rotation2d dumperFacing = robotPose.getRotation().plus(Rotation2d.kPi);
    ChassisSpeeds fieldSpeeds = driveSimulation.getDriveTrainSimulatedChassisSpeedsFieldRelative();
    double launchSpeedMps =
        Math.max(
            shooterRps * FieldSimulationConstants.DumperMetersPerSecondPerRps,
            FieldSimulationConstants.DumperMinLaunchSpeedMps);

    int launched = 0;
    for (Translation2d lateralOffset : dumperSlotOffsets()) {
      if (!intakeSimulation.obtainGamePieceFromIntake()) {
        break;
      }
      launchSingleFuel(
          robotPose, fieldSpeeds, dumperFacing, hoodDegs, launchSpeedMps, lateralOffset);
      launched++;
    }

    Logger.recordOutput("FieldSimulation/VolleyLaunched", launched);
    Logger.recordOutput("FieldSimulation/LaunchedFuel", launched > 0);
    Logger.recordOutput("FieldSimulation/LaunchSpeedMps", launchSpeedMps);
    return launched > 0;
  }

  private static Translation2d[] dumperSlotOffsets() {
    double spacing = FieldSimulationConstants.DumperLateralSpacingMeters;
    return new Translation2d[] {
      new Translation2d(0.0, spacing), new Translation2d(0.0, 0.0), new Translation2d(0.0, -spacing)
    };
  }

  private void launchSingleFuel(
      Pose2d robotPose,
      ChassisSpeeds fieldSpeeds,
      Rotation2d dumperFacing,
      double hoodDegs,
      double launchSpeedMps,
      Translation2d lateralOffset) {
    launchFuelProjectile(
        robotPose.getTranslation(),
        ShooterConstants.ShooterOffset.plus(lateralOffset),
        fieldSpeeds,
        dumperFacing,
        Meters.of(FieldSimulationConstants.DumperExitHeightMeters),
        MetersPerSecond.of(launchSpeedMps),
        Degrees.of(hoodDegs),
        true);
  }

  private void launchFuelProjectile(
      Translation2d robotPosition,
      Translation2d shooterOffsetOnRobot,
      ChassisSpeeds fieldSpeeds,
      Rotation2d shooterFacing,
      Distance initialHeight,
      LinearVelocity launchSpeed,
      Angle hoodAngle,
      boolean trackHubHit) {
    RebuiltFuelOnFly projectile =
        new RebuiltFuelOnFly(
            robotPosition,
            shooterOffsetOnRobot,
            fieldSpeeds,
            shooterFacing,
            initialHeight,
            launchSpeed,
            hoodAngle);

    SimulatedArena.getInstance().addGamePieceProjectile(projectile);
  }

  private void spawnDefaultFuel() {
    var arena = SimulatedArena.getInstance();
    arena.addGamePiece(new RebuiltFuelOnField(new Translation2d(2.0, 2.0)));
    arena.addGamePiece(new RebuiltFuelOnField(new Translation2d(2.0, 6.0)));
    arena.addGamePiece(new RebuiltFuelOnField(new Translation2d(4.5, 4.0)));
    arena.addGamePiece(new RebuiltFuelOnField(new Translation2d(6.0, 2.5)));
    arena.addGamePiece(new RebuiltFuelOnField(new Translation2d(6.0, 5.5)));
  }

  private void logFieldTelemetry() {
    Logger.recordOutput(
        "FieldSimulation/RobotPosition", driveSimulation.getSimulatedDriveTrainPose());
    Logger.recordOutput(
        "FieldSimulation/Fuel",
        SimulatedArena.getInstance().getGamePiecesArrayByType(FieldSimulationConstants.FUEL_TYPE));
    Logger.recordOutput("FieldSimulation/FuelInHopper", intakeSimulation.getGamePiecesAmount());
    Logger.recordOutput("FieldSimulation/HopperCapacity", FieldSimulationConstants.HopperCapacity);
    Logger.recordOutput("FieldSimulation/IntakeRunning", intakeSimulation.isRunning());
  }

  private static Pose3d[] toPose3dArray(List<Pose3d> poses) {
    return poses.toArray(Pose3d[]::new);
  }

  private static DriveTrainSimulationConfig createMapleSimConfig() {
    var module = TunerConstants.FrontLeft;
    return DriveTrainSimulationConfig.Default()
        .withRobotMass(Kilograms.of(ROBOT_MASS_KG))
        .withCustomModuleTranslations(CommandSwerveDrivetrain.getModuleTranslations())
        .withGyro(COTS.ofPigeon2())
        .withSwerveModule(
            new SwerveModuleSimulationConfig(
                DCMotor.getKrakenX60(1),
                DCMotor.getFalcon500(1),
                module.DriveMotorGearRatio,
                module.SteerMotorGearRatio,
                Volts.of(module.DriveFrictionVoltage),
                Volts.of(module.SteerFrictionVoltage),
                Meters.of(module.WheelRadius),
                KilogramSquareMeters.of(module.SteerInertia),
                WHEEL_COF));
  }
}
