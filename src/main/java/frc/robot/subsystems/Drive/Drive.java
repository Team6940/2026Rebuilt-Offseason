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

package frc.robot.subsystems.Drive;

import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.CANBus;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPoint;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.Mode;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.SuperStructure;
import frc.robot.subsystems.SuperStructure.IntakeMode;
import frc.robot.subsystems.Vision.LimelightHelpers;
import frc.robot.util.LocalADStarAK;
import java.util.List;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Drive extends SubsystemBase {
  private static Drive instance;

  public static Drive getInstance() {
    return instance;
  }

  // TunerConstants doesn't include these constants, so they are declared locally
  static final double ODOMETRY_FREQUENCY =
      new CANBus(TunerConstants.DrivetrainConstants.CANBusName).isNetworkFD() ? 250.0 : 200.0;
  public static final double DRIVE_BASE_RADIUS =
      Math.max(
          Math.max(
              Math.hypot(TunerConstants.FrontLeft.LocationX, TunerConstants.FrontLeft.LocationY),
              Math.hypot(TunerConstants.FrontRight.LocationX, TunerConstants.FrontRight.LocationY)),
          Math.max(
              Math.hypot(TunerConstants.BackLeft.LocationX, TunerConstants.BackLeft.LocationY),
              Math.hypot(TunerConstants.BackRight.LocationX, TunerConstants.BackRight.LocationY)));

  // PathPlanner config constants
  private static final double ROBOT_MASS_KG = 74.088;
  private static final double ROBOT_MOI = 6.883;
  private static final double WHEEL_COF = 2.255;
  private static final RobotConfig PP_CONFIG =
      new RobotConfig(
          ROBOT_MASS_KG,
          ROBOT_MOI,
          new ModuleConfig(
              TunerConstants.FrontLeft.WheelRadius,
              TunerConstants.kSpeedAt12Volts.in(MetersPerSecond),
              WHEEL_COF,
              DCMotor.getKrakenX60Foc(1)
                  .withReduction(TunerConstants.FrontLeft.DriveMotorGearRatio),
              TunerConstants.FrontLeft.SlipCurrent,
              1),
          getModuleTranslations());

  public static final DriveTrainSimulationConfig mapleSimConfig =
      DriveTrainSimulationConfig.Default()
          .withRobotMass(Kilograms.of(ROBOT_MASS_KG))
          .withCustomModuleTranslations(getModuleTranslations())
          .withGyro(COTS.ofPigeon2())
          .withSwerveModule(
              new SwerveModuleSimulationConfig(
                  DCMotor.getKrakenX60(1),
                  DCMotor.getFalcon500(1),
                  TunerConstants.FrontLeft.DriveMotorGearRatio,
                  TunerConstants.FrontLeft.SteerMotorGearRatio,
                  Volts.of(TunerConstants.FrontLeft.DriveFrictionVoltage),
                  Volts.of(TunerConstants.FrontLeft.SteerFrictionVoltage),
                  Meters.of(TunerConstants.FrontLeft.WheelRadius),
                  KilogramSquareMeters.of(TunerConstants.FrontLeft.SteerInertia),
                  WHEEL_COF));

  // Simulation helper: you can generate simulation-friendly module constants
  // using PhoenixUtil.regulateModuleConstantForSimulation(TunerConstants.FrontLeft) etc.
  static final Lock odometryLock = new ReentrantLock();
  private final GyroIO gyroIO;
  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
  private final Module[] modules = new Module[4]; // FL, FR, BL, BR
  private final SysIdRoutine sysId;
  private final Alert gyroDisconnectedAlert =
      new Alert("Disconnected gyro, using kinematics as fallback.", AlertType.kError);

  private final SwerveDriveKinematics kinematics =
      new SwerveDriveKinematics(getModuleTranslations());
  private Rotation2d rawGyroRotation = new Rotation2d();
  private final SwerveModulePosition[] lastModulePositions = // For delta tracking
      new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
      };
  private final SwerveDrivePoseEstimator poseEstimator =
      new SwerveDrivePoseEstimator(kinematics, rawGyroRotation, lastModulePositions, new Pose2d());

  private final Field2d field2d = new Field2d();

  // PID Controllers for autoMoveToPose
  private final PIDController xController;
  private final PIDController yController;
  private final ProfiledPIDController thetaController;
  private final ProfiledPIDController fieldCentricAngleController;

  private PathPlannerPath trenchPathAtl;
  private PathPlannerPath trenchPathAtr;
  private Rotation2d snappedSquareEdge = new Rotation2d();

  private void initializeAutoMoveToPoseControllers() {
    // Initialize X and Y PID controllers
    xController.setPID(DriveConstants.MOVE_TO_X_KP, 0.0, DriveConstants.MOVE_TO_X_KD);
    yController.setPID(DriveConstants.MOVE_TO_Y_KP, 0.0, DriveConstants.MOVE_TO_Y_KD);

    // Initialize theta ProfiledPID controller
    thetaController.setPID(DriveConstants.MOVE_TO_THETA_KP, 0.0, DriveConstants.MOVE_TO_THETA_KD);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
    thetaController.setConstraints(
        new TrapezoidProfile.Constraints(
            DriveConstants.ANGLE_MAX_VELOCITY, DriveConstants.ANGLE_MAX_ACCELERATION));

    // Set tolerances
    xController.setTolerance(DriveConstants.MOVE_TO_POSITION_TOLERANCE_METERS);
    yController.setTolerance(DriveConstants.MOVE_TO_POSITION_TOLERANCE_METERS);
    thetaController.setTolerance(
        Units.degreesToRadians(DriveConstants.MOVE_TO_ANGLE_TOLERANCE_DEGREES));
  }

  public Drive(
      GyroIO gyroIO,
      ModuleIO flModuleIO,
      ModuleIO frModuleIO,
      ModuleIO blModuleIO,
      ModuleIO brModuleIO) {
    instance = this;
    SmartDashboard.putData("Field", field2d);
    this.gyroIO = gyroIO;
    modules[0] = new Module(flModuleIO, 0, TunerConstants.FrontLeft);
    modules[1] = new Module(frModuleIO, 1, TunerConstants.FrontRight);
    modules[2] = new Module(blModuleIO, 2, TunerConstants.BackLeft);
    modules[3] = new Module(brModuleIO, 3, TunerConstants.BackRight);

    // Initialize PID controllers for autoMoveToPose
    xController = new PIDController(0, 0, 0);
    yController = new PIDController(0, 0, 0);
    thetaController = new ProfiledPIDController(0, 0, 0, new TrapezoidProfile.Constraints(0, 0));
    initializeAutoMoveToPoseControllers();

    // Initialize field centric angle controller
    fieldCentricAngleController =
        new ProfiledPIDController(
            Constants.DriveConstants.ANGLE_KP,
            0.0,
            Constants.DriveConstants.ANGLE_KD,
            new TrapezoidProfile.Constraints(
                Constants.DriveConstants.ANGLE_MAX_VELOCITY,
                Constants.DriveConstants.ANGLE_MAX_ACCELERATION));
    fieldCentricAngleController.enableContinuousInput(-Math.PI, Math.PI);

    // Usage reporting for swerve template
    HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_AdvantageKit);

    // Start odometry thread
    PhoenixOdometryThread.getInstance().start();

    // Configure AutoBuilder for PathPlanner
    AutoBuilder.configure(
        this::getPose,
        this::setPose,
        this::getChassisSpeeds,
        this::runVelocity,
        new PPHolonomicDriveController(
            new PIDConstants(
                DriveConstants.PP_TRANSLATION_KP, 0.0, DriveConstants.PP_TRANSLATION_KD),
            new PIDConstants(DriveConstants.PP_ROTATION_KP, 0.0, DriveConstants.PP_ROTATION_KD)),
        PP_CONFIG,
        () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
        this);
    Pathfinding.setPathfinder(new LocalADStarAK());
    loadTrenchPaths();
    PathPlannerLogging.setLogActivePathCallback(
        (activePath) -> {
          Logger.recordOutput(
              "Odometry/Trajectory", activePath.toArray(new Pose2d[activePath.size()]));
        });
    PathPlannerLogging.setLogTargetPoseCallback(
        (targetPose) -> {
          Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose);
        });

    // Configure SysId
    sysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("Drive/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> runCharacterization(voltage.in(Volts)), null, this));
  }

  @Override
  public void periodic() {
    odometryLock.lock(); // Prevents odometry updates while reading data
    gyroIO.updateInputs(gyroInputs);
    Logger.processInputs("Drive/Gyro", gyroInputs);
    for (var module : modules) {
      module.periodic();
    }
    odometryLock.unlock();

    // Stop moving when disabled
    if (DriverStation.isDisabled()) {
      for (var module : modules) {
        module.stop();
      }
    }

    // Log empty setpoint states when disabled
    if (DriverStation.isDisabled()) {
      Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleState[] {});
      Logger.recordOutput("SwerveStates/SetpointsOptimized", new SwerveModuleState[] {});
    }

    // Update odometry
    double[] sampleTimestamps =
        modules[0].getOdometryTimestamps(); // All signals are sampled together
    int sampleCount = sampleTimestamps.length;
    for (int i = 0; i < sampleCount; i++) {
      // Read wheel positions and deltas from each module
      SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
      SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];
      for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
        modulePositions[moduleIndex] = modules[moduleIndex].getOdometryPositions()[i];
        moduleDeltas[moduleIndex] =
            new SwerveModulePosition(
                modulePositions[moduleIndex].distanceMeters
                    - lastModulePositions[moduleIndex].distanceMeters,
                modulePositions[moduleIndex].angle);
        lastModulePositions[moduleIndex] = modulePositions[moduleIndex];
      }

      // Update gyro angle
      if (gyroInputs.connected) {
        // Use the real gyro angle
        rawGyroRotation = gyroInputs.odometryYawPositions[i];
      } else {
        // Use the angle delta from the kinematics and module deltas
        Twist2d twist = kinematics.toTwist2d(moduleDeltas);
        rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
      }

      // Apply update
      poseEstimator.updateWithTime(sampleTimestamps[i], rawGyroRotation, modulePositions);
    }

    // Update gyro alert
    gyroDisconnectedAlert.set(!gyroInputs.connected && Constants.currentMode != Mode.SIM);

    // Logging data
    processLog();
  }

  public void processLog() {
    field2d.setRobotPose(getPose());
    Logger.recordOutput("Odometry/Robot", getPose());
    Logger.recordOutput("Drive/ChassisSpeeds", getChassisSpeeds());
  }

  /**
   * MegaTag2 requires fresh gyro orientation over NetworkTables before reading {@code botpose}.
   * Called by {@link frc.robot.subsystems.Vision.VisionSubsystem} for each Limelight.
   */
  public void applyLimelightGyroForMegaTag2(String limelightName) {
    LimelightHelpers.SetRobotOrientation(
        limelightName,
        getPose().getRotation().getDegrees(),
        Math.toDegrees(getChassisSpeeds().omegaRadiansPerSecond),
        gyroInputs.pitchPosition.getDegrees(),
        0,
        gyroInputs.rollPosition.getDegrees(),
        0);
  }

  public static Translation2d getLinearVelocityFromJoysticks(double x, double y) {
    // Apply deadband
    double linearMagnitude =
        MathUtil.applyDeadband(Math.hypot(x, y), Constants.DriveConstants.DEADBAND);
    Rotation2d linearDirection = new Rotation2d(Math.atan2(y, x));

    // Square magnitude for more precise control
    linearMagnitude = linearMagnitude * linearMagnitude;

    // Return new linear velocity
    return new Pose2d(new Translation2d(), linearDirection)
        .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
        .getTranslation();
  }

  /**
   * Runs the drive at the desired velocity.
   *
   * @param speeds Speeds in meters/sec
   */
  public void runVelocity(ChassisSpeeds speeds) {
    // Calculate module setpoints
    ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02);
    SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(discreteSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, TunerConstants.kSpeedAt12Volts);

    // Log unoptimized setpoints and setpoint speeds
    Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
    Logger.recordOutput("SwerveChassisSpeeds/Setpoints", discreteSpeeds);

    // Send setpoints to modules
    for (int i = 0; i < 4; i++) {
      modules[i].runSetpoint(setpointStates[i]);
    }

    // Log optimized setpoints (runSetpoint mutates each state)
    Logger.recordOutput("SwerveStates/SetpointsOptimized", setpointStates);
  }

  public void runFieldRelativeVelocity(ChassisSpeeds speeds) {
    ChassisSpeeds fieldRelativeVelocity =
        ChassisSpeeds.fromFieldRelativeSpeeds(speeds, this.getRotation());
    runVelocity(fieldRelativeVelocity);
  }

  public void driveFieldCentric(
      DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier omegaSupplier) {
    // Get linear velocity
    Translation2d linearVelocity =
        getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());

    // Apply deadband to angular velocity
    double omega =
        MathUtil.applyDeadband(omegaSupplier.getAsDouble(), Constants.DriveConstants.DEADBAND);

    // Square the angular velocity for finer control
    omega = Math.copySign(omega * omega, omega);

    ChassisSpeeds speeds =
        new ChassisSpeeds(
            linearVelocity.getX() * this.getMaxLinearSpeedMetersPerSec(),
            linearVelocity.getY() * this.getMaxLinearSpeedMetersPerSec(),
            omega * this.getMaxAngularSpeedRadPerSec());
    boolean isFlipped =
        DriverStation.getAlliance().isPresent()
            && DriverStation.getAlliance().get() == Alliance.Red;
    this.runVelocity(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            speeds,
            isFlipped ? this.getRotation().plus(new Rotation2d(Math.PI)) : this.getRotation()));
  }

  /**
   * Field-centric drive with custom max speed. Allows you to scale the maximum linear and angular
   * speeds independently.
   *
   * @param xSupplier X-axis joystick input (-1 to 1, forward positive)
   * @param ySupplier Y-axis joystick input (-1 to 1, left positive)
   * @param omegaSupplier Rotational joystick input (-1 to 1, CCW positive)
   * @param maxLinearSpeed Maximum linear speed in meters per second
   * @param maxAngularSpeed Maximum angular speed in radians per second
   */
  public void driveFieldCentricWithMaxSpeed(
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier,
      double maxLinearSpeed,
      double maxAngularSpeed) {
    // Get linear velocity
    Translation2d linearVelocity =
        getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());

    // Apply deadband to angular velocity
    double omega =
        MathUtil.applyDeadband(omegaSupplier.getAsDouble(), Constants.DriveConstants.DEADBAND);

    // Square the angular velocity for finer control
    omega = Math.copySign(omega * omega, omega);

    ChassisSpeeds speeds =
        new ChassisSpeeds(
            linearVelocity.getX() * maxLinearSpeed,
            linearVelocity.getY() * maxLinearSpeed,
            omega * maxAngularSpeed);
    boolean isFlipped =
        DriverStation.getAlliance().isPresent()
            && DriverStation.getAlliance().get() == Alliance.Red;
    this.runVelocity(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            speeds,
            isFlipped ? this.getRotation().plus(new Rotation2d(Math.PI)) : this.getRotation()));
  }

  /**
   * Field relative drive using joystick for linear control and PID for angular control. Possible
   * use cases include snapping to an angle, aiming at a vision target, or controlling absolute
   * rotation with a joystick.
   */
  public void driveFieldCentricAtAngle(
      DoubleSupplier xSupplier, DoubleSupplier ySupplier, Supplier<Rotation2d> rotationSupplier) {
    // Get linear velocity
    Translation2d linearVelocity =
        getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());

    // Calculate angular speed
    double omega =
        fieldCentricAngleController.calculate(
            getRotation().getRadians(), rotationSupplier.get().getRadians());

    // Convert to field relative speeds & send command
    ChassisSpeeds speeds =
        new ChassisSpeeds(
            linearVelocity.getX() * getMaxLinearSpeedMetersPerSec(),
            linearVelocity.getY() * getMaxLinearSpeedMetersPerSec(),
            omega);
    boolean isFlipped =
        DriverStation.getAlliance().isPresent()
            && DriverStation.getAlliance().get() == Alliance.Red;
    runVelocity(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            speeds, isFlipped ? getRotation().plus(new Rotation2d(Math.PI)) : getRotation()));
  }

  /** Runs the drive in a straight line with the specified drive output. */
  public void runCharacterization(double output) {
    for (int i = 0; i < 4; i++) {
      modules[i].runCharacterization(output);
    }
  }

  /** Stops the drive. */
  public void stop() {
    runVelocity(new ChassisSpeeds());
  }

  /**
   * Stops the drive and turns the modules to an X arrangement to resist movement. The modules will
   * return to their normal orientations the next time a nonzero velocity is requested.
   */
  public void stopWithX() {
    Rotation2d[] headings = new Rotation2d[4];
    for (int i = 0; i < 4; i++) {
      headings[i] = getModuleTranslations()[i].getAngle();
    }
    kinematics.resetHeadings(headings);
    stop();
  }

  /** Returns a command to run a quasistatic test in the specified direction. */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return run(() -> runCharacterization(0.0))
        .withTimeout(1.0)
        .andThen(sysId.quasistatic(direction));
  }

  /** Returns a command to run a dynamic test in the specified direction. */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return run(() -> runCharacterization(0.0)).withTimeout(1.0).andThen(sysId.dynamic(direction));
  }

  /** Returns the module states (turn angles and drive velocities) for all of the modules. */
  @AutoLogOutput(key = "SwerveStates/Measured")
  private SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getState();
    }
    return states;
  }

  /** Returns the module positions (turn angles and drive positions) for all of the modules. */
  private SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] states = new SwerveModulePosition[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getPosition();
    }
    return states;
  }

  /** Returns the measured chassis speeds of the robot. */
  @AutoLogOutput(key = "SwerveChassisSpeeds/Measured")
  public ChassisSpeeds getChassisSpeeds() {
    return kinematics.toChassisSpeeds(getModuleStates());
  }

  /** Returns the robot's current rotational velocity in radians per second. */
  public double getRobotOmegaRadPerSec() {
    return getChassisSpeeds().omegaRadiansPerSecond;
  }

  /** Returns the position of each module in radians. */
  public double[] getWheelRadiusCharacterizationPositions() {
    double[] values = new double[4];
    for (int i = 0; i < 4; i++) {
      values[i] = modules[i].getWheelRadiusCharacterizationPosition();
    }
    return values;
  }

  /** Returns the average velocity of the modules in rotations/sec (Phoenix native units). */
  public double getFFCharacterizationVelocity() {
    double output = 0.0;
    for (int i = 0; i < 4; i++) {
      output += modules[i].getFFCharacterizationVelocity() / 4.0;
    }
    return output;
  }

  /** Returns the current odometry pose. */
  @AutoLogOutput(key = "Odometry/Robot")
  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  /** Returns the current odometry rotation. */
  public Rotation2d getRotation() {
    return getPose().getRotation();
  }



  /** Resets the current odometry pose. */
  public void setPose(Pose2d pose) {
    poseEstimator.resetPosition(rawGyroRotation, getModulePositions(), pose);
  }

  /** Adds a new timestamped vision measurement. */
  public void addVisionMeasurement(
      Pose2d visionRobotPoseMeters,
      double timestampSeconds,
      Matrix<N3, N1> visionMeasurementStdDevs) {
    poseEstimator.addVisionMeasurement(
        visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
  }

  /** Returns the maximum linear speed in meters per sec. */
  public double getMaxLinearSpeedMetersPerSec() {
    return TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
  }

  /** Returns the maximum angular speed in radians per sec. */
  public double getMaxAngularSpeedRadPerSec() {
    return getMaxLinearSpeedMetersPerSec() / DRIVE_BASE_RADIUS;
  }

  /** Returns an array of module translations. */
  public static Translation2d[] getModuleTranslations() {
    return new Translation2d[] {
      new Translation2d(TunerConstants.FrontLeft.LocationX, TunerConstants.FrontLeft.LocationY),
      new Translation2d(TunerConstants.FrontRight.LocationX, TunerConstants.FrontRight.LocationY),
      new Translation2d(TunerConstants.BackLeft.LocationX, TunerConstants.BackLeft.LocationY),
      new Translation2d(TunerConstants.BackRight.LocationX, TunerConstants.BackRight.LocationY)
    };
  }

  public Command followPPPath(String pathName) {
    return AutoBuilder.followPath(generatePPPath(pathName));
  }

  public PathPlannerPath generatePPPath(String pathName) {
    try {
      return PathPlannerPath.fromPathFile(pathName);
    } catch (Exception e) {
      e.printStackTrace();
      return null;
    }
  }

  /**
   * Automatically moves the robot to the target pose using separate PID controllers for X, Y, and
   * theta. This method should be called periodically (e.g., in a command's execute() method) until
   * the robot reaches the target pose.
   *
   * @param targetPose The desired pose to move to (field-relative)
   */
  public void autoMoveToPose(Pose2d targetPose) {
    Pose2d currentPose = getPose();

    double xVelocity = xController.calculate(currentPose.getX(), targetPose.getX());
    double yVelocity = yController.calculate(currentPose.getY(), targetPose.getY());
    double omega =
        thetaController.calculate(
            currentPose.getRotation().getRadians(), targetPose.getRotation().getRadians());

    // Clamp linear velocity by vector magnitude
    Translation2d linear = new Translation2d(xVelocity, yVelocity);
    double maxV = getMaxLinearSpeedMetersPerSec();
    if (linear.getNorm() > maxV) {
      linear = linear.times(maxV / linear.getNorm());
    }

    Logger.recordOutput("Drive/AutoMoveToPose/Target", targetPose);
    Logger.recordOutput(
        "AutoMoveToPose/CommandedSpeeds", new ChassisSpeeds(linear.getX(), linear.getY(), omega));

    if (atTargetPose()) {
      stop();
      return;
    }

    runFieldRelativeVelocity(new ChassisSpeeds(linear.getX(), linear.getY(), omega));
  }

  /**
   * Returns whether the robot is at the target pose within specified tolerances.
   *
   * @return true if the robot is within tolerance of the target pose
   */
  public boolean atTargetPose() {
    return xController.atSetpoint() && yController.atSetpoint() && thetaController.atGoal();
  }

  // ---------------------------------------------------------------------------
  // Shared-control holonomic lane assist (AutoTrench + AutoIntake)
  // ---------------------------------------------------------------------------

  private void loadTrenchPaths() {
    trenchPathAtl = allianceFlipPath(generatePPPath(TrenchLane.ATLtoNTL.pathName));
    trenchPathAtr = allianceFlipPath(generatePPPath(TrenchLane.ATRtoNTR.pathName));
  }

  private PathPlannerPath allianceFlipPath(PathPlannerPath path) {
    if (path == null) {
      return null;
    }
    return isRedAlliance() ? path.flipPath() : path;
  }

  private static boolean isRedAlliance() {
    return DriverStation.getAlliance().isPresent()
        && DriverStation.getAlliance().get() == Alliance.Red;
  }

  /**
   * Field-relative shared-control trench drive. Caller must only invoke this while {@link
   * frc.robot.subsystems.SuperStructure.DriveMode#HYBRID_TRENCH} is active (e.g. via {@link
   * frc.robot.commands.DriveAutoTrenchCommand}).
   */
  public void driveAutoTrench(
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier,
      double maxLinearSpeed,
      double maxAngularSpeed) {
    IntakeMode intakeMode = SuperStructure.getInstance().getIntakeMode();

    Translation2d driverLinear =
        getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble())
            .times(maxLinearSpeed);

    double omegaInput =
        MathUtil.applyDeadband(omegaSupplier.getAsDouble(), DriveConstants.DEADBAND);
    double stickOmega = Math.copySign(omegaInput * omegaInput, omegaInput) * maxAngularSpeed;

    AutoTrenchReference trenchRef = computeAutoTrenchReference(getPose(), driverLinear);
    Translation2d fieldLinear = blendDriverInput(driverLinear, trenchRef, maxLinearSpeed);
    Logger.recordOutput("AutoTrench/Lane", trenchRef.lane().name());
    Logger.recordOutput("AutoTrench/PathTraversalSign", trenchRef.pathTraversalSign());
    Logger.recordOutput("AutoTrench/GuidanceVector", trenchRef.guidanceVector());
    Logger.recordOutput(
        "AutoTrench/Lookahead", new Pose2d(trenchRef.lookaheadPoint(), new Rotation2d()));
    Logger.recordOutput(
        "AutoTrench/Nearest", new Pose2d(trenchRef.nearestPoint(), new Rotation2d()));

    Rotation2d desiredFacing = getDesiredFacing(fieldLinear, intakeMode);

    double omega;
    if (fieldLinear.getNorm() > 0.08) {
      omega = calculateOmega(desiredFacing);
    } else {
      omega = stickOmega;
    }

    ChassisSpeeds speeds = new ChassisSpeeds(fieldLinear.getX(), fieldLinear.getY(), omega);
    runVelocity(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            speeds,
            isRedAlliance() ? getRotation().plus(new Rotation2d(Math.PI)) : getRotation()));
    Logger.recordOutput("Drive/DesiredFacing", desiredFacing);
    Logger.recordOutput("Drive/BlendedFieldLinear", fieldLinear);
  }

  /**
   * Selects lane, nearest spline point, and robotPose→lookahead guidance vector.
   *
   * @param driverFieldLinear field-relative linear intent from the left stick (m/s), same frame as
   *     {@link #driveAutoTrench}
   */
  public AutoTrenchReference computeAutoTrenchReference(
      Pose2d robotPose, Translation2d driverFieldLinear) {
    TrenchLane lane = selectTrenchLane(robotPose, driverFieldLinear);
    PathPlannerPath path = getTrenchPath(lane);
    List<PathPoint> points = path.getAllPathPoints();

    int nearestIndex = findNearestPointIndex(robotPose.getTranslation(), points);
    PathPoint nearest = points.get(nearestIndex);
    double distanceAlong = nearest.distanceAlongPath;

    double pathLength = points.get(points.size() - 1).distanceAlongPath;
    double traversalSign =
        pathTraversalSign(points, distanceAlong, driverFieldLinear, robotPose);
    double lookaheadDistance =
        MathUtil.clamp(
            distanceAlong + traversalSign * DriveConstants.TRENCH_LOOKAHEAD_METERS,
            0.0,
            pathLength);
    Translation2d lookaheadPoint = sampleTranslationAtDistance(points, lookaheadDistance);

    Translation2d guidanceVector = lookaheadPoint.minus(robotPose.getTranslation());

    return new AutoTrenchReference(
        lane,
        guidanceVector,
        lookaheadPoint,
        nearest.position,
        distanceAlong,
        traversalSign,
        robotPose);
  }

  /**
   * +1 = lookahead moves with path parameterization; -1 = opposite (reverse trench entry). Uses
   * field-relative driver intent when moving; otherwise falls back to geometry near path ends.
   */
  private static double pathTraversalSign(
      List<PathPoint> points,
      double distanceAlong,
      Translation2d driverFieldLinear,
      Pose2d robotPose) {
    Translation2d tangent = pathTangentAtDistance(points, distanceAlong);
    if (driverFieldLinear.getNorm() > DriveConstants.TRENCH_DRIVER_INTENT_THRESHOLD) {
      Translation2d driverDir = driverFieldLinear.div(driverFieldLinear.getNorm());
      double alongTangent = driverDir.dot(tangent);
      if (alongTangent < -0.1) {
        return -1.0;
      }
      if (alongTangent > 0.1) {
        return 1.0;
      }
    }

    double pathLength = points.get(points.size() - 1).distanceAlongPath;
    Translation2d pathStart = points.get(0).position;
    Translation2d pathEnd = points.get(points.size() - 1).position;
    double distToStart = robotPose.getTranslation().getDistance(pathStart);
    double distToEnd = robotPose.getTranslation().getDistance(pathEnd);

    if (distToStart + distToEnd < 1e-6) {
      return 1.0;
    }
    if (distanceAlong < pathLength * 0.35 && distToStart < distToEnd) {
      return -1.0;
    }
    if (distanceAlong > pathLength * 0.65 && distToEnd < distToStart) {
      return 1.0;
    }
    return 1.0;
  }

  private static Translation2d pathTangentAtDistance(List<PathPoint> points, double distanceAlong) {
    double pathLength = points.get(points.size() - 1).distanceAlongPath;
    double eps = DriveConstants.TRENCH_TANGENT_EPSILON;
    Translation2d ahead =
        sampleTranslationAtDistance(points, Math.min(distanceAlong + eps, pathLength));
    Translation2d behind =
        sampleTranslationAtDistance(points, Math.max(distanceAlong - eps, 0.0));
    Translation2d tangent = ahead.minus(behind);
    if (tangent.getNorm() < 1e-6) {
      return new Translation2d(1.0, 0.0);
    }
    return tangent.div(tangent.getNorm());
  }

  private TrenchLane selectTrenchLane(Pose2d robotPose, Translation2d driverFieldLinear) {
    if (driverFieldLinear.getNorm() <= DriveConstants.TRENCH_DRIVER_INTENT_THRESHOLD) {
      double distAtl = distanceToPath(robotPose, trenchPathAtl);
      double distAtr = distanceToPath(robotPose, trenchPathAtr);
      return distAtl <= distAtr ? TrenchLane.ATLtoNTL : TrenchLane.ATRtoNTR;
    }

    double scoreAtl = trenchLaneAlignmentScore(robotPose, trenchPathAtl, driverFieldLinear);
    double scoreAtr = trenchLaneAlignmentScore(robotPose, trenchPathAtr, driverFieldLinear);
    if (Math.abs(scoreAtl - scoreAtr) < 0.05) {
      double distAtl = distanceToPath(robotPose, trenchPathAtl);
      double distAtr = distanceToPath(robotPose, trenchPathAtr);
      return distAtl <= distAtr ? TrenchLane.ATLtoNTL : TrenchLane.ATRtoNTR;
    }
    return scoreAtl >= scoreAtr ? TrenchLane.ATLtoNTL : TrenchLane.ATRtoNTR;
  }

  private double trenchLaneAlignmentScore(
      Pose2d robotPose, PathPlannerPath path, Translation2d driverFieldLinear) {
    if (path == null) {
      return Double.NEGATIVE_INFINITY;
    }
    List<PathPoint> points = path.getAllPathPoints();
    int idx = findNearestPointIndex(robotPose.getTranslation(), points);
    Translation2d tangent = pathTangentAtDistance(points, points.get(idx).distanceAlongPath);
    Translation2d driverDir = driverFieldLinear.div(driverFieldLinear.getNorm());
    double alignment = Math.abs(driverDir.dot(tangent));
    double crossTrack = robotPose.getTranslation().getDistance(points.get(idx).position);
    return alignment - crossTrack * 0.25;
  }

  private double distanceToPath(Pose2d robotPose, PathPlannerPath path) {
    if (path == null) {
      return Double.MAX_VALUE;
    }
    List<PathPoint> points = path.getAllPathPoints();
    int idx = findNearestPointIndex(robotPose.getTranslation(), points);
    return robotPose.getTranslation().getDistance(points.get(idx).position);
  }

  private PathPlannerPath getTrenchPath(TrenchLane lane) {
    return lane == TrenchLane.ATLtoNTL ? trenchPathAtl : trenchPathAtr;
  }

  private static int findNearestPointIndex(Translation2d robot, List<PathPoint> points) {
    int best = 0;
    double bestDist = Double.MAX_VALUE;
    for (int i = 0; i < points.size(); i++) {
      double d = robot.getDistance(points.get(i).position);
      if (d < bestDist) {
        bestDist = d;
        best = i;
      }
    }
    return best;
  }

  private static Translation2d sampleTranslationAtDistance(
      List<PathPoint> points, double distanceMeters) {
    if (points.isEmpty()) {
      return new Translation2d();
    }
    if (distanceMeters <= points.get(0).distanceAlongPath) {
      return points.get(0).position;
    }
    PathPoint last = points.get(points.size() - 1);
    if (distanceMeters >= last.distanceAlongPath) {
      return last.position;
    }
    for (int i = 1; i < points.size(); i++) {
      PathPoint a = points.get(i - 1);
      PathPoint b = points.get(i);
      if (distanceMeters <= b.distanceAlongPath) {
        double span = b.distanceAlongPath - a.distanceAlongPath;
        double t =
            span < 1e-9
                ? 0.0
                : (distanceMeters - a.distanceAlongPath) / span;
        return a.position.interpolate(b.position, t);
      }
    }
    return last.position;
  }

  /**
   * Continuous shared-control blend: strong driver/guidance misalignment reduces assist; aligned
   * motion is projected toward the guidance direction.
   */
  public Translation2d blendDriverInput(
      Translation2d driverVelocity, AutoTrenchReference ref, double maxLinearSpeed) {
    Translation2d guidance = ref.guidanceVector();
    double guidanceNorm = guidance.getNorm();
    if (guidanceNorm < 1e-6) {
      return driverVelocity;
    }

    Translation2d guidanceDir = guidance.div(guidanceNorm);
    double driverNorm = driverVelocity.getNorm();

    Translation2d driverDir =
        driverNorm > 1e-6 ? driverVelocity.div(driverNorm) : guidanceDir;

    double alignment = driverDir.dot(guidanceDir);
    double alignment01 =
        MathUtil.clamp(
            (alignment - DriveConstants.TRENCH_MIN_ALIGNMENT)
                / (1.0 - DriveConstants.TRENCH_MIN_ALIGNMENT),
            0.0,
            1.0);
    double assistWeight = alignment01 * DriveConstants.TRENCH_MAX_ASSIST;

    double alongGuidance = driverVelocity.dot(guidanceDir);
    Translation2d projected = guidanceDir.times(alongGuidance);

    Translation2d assistPull = guidanceDir.times(maxLinearSpeed * assistWeight);

    return driverVelocity
        .interpolate(projected, assistWeight)
        .plus(assistPull.times(0.35 * assistWeight));
  }

  /**
   * Final blended velocity heading for rotation. Hybrid intake uses velocity direction; otherwise
   * a square robot edge (0/90/180/270°) with hysteresis.
   */
  public Rotation2d getDesiredFacing(Translation2d blendedVelocity, IntakeMode intakeMode) {
    if (blendedVelocity.getNorm() < 0.08) {
      return getRotation();
    }
    Rotation2d velocityHeading =
        new Rotation2d(Math.atan2(blendedVelocity.getY(), blendedVelocity.getX()));
    if (intakeMode == IntakeMode.HYBRID) {
      return velocityHeading;
    }
    return snapToNearestEdge(velocityHeading);
  }

  /** Snaps to 0/90/180/270° field headings; only switches when improvement exceeds hysteresis. */
  public Rotation2d snapToNearestEdge(Rotation2d velocityHeading) {
    Rotation2d[] edges = {
      new Rotation2d(),
      new Rotation2d(Math.PI / 2),
      new Rotation2d(Math.PI),
      new Rotation2d(-Math.PI / 2)
    };

    Rotation2d bestEdge = edges[0];
    double bestError = Double.MAX_VALUE;
    for (Rotation2d edge : edges) {
      double error =
          Math.abs(velocityHeading.minus(edge).getRadians());
      if (error < bestError) {
        bestError = error;
        bestEdge = edge;
      }
    }

    double currentError = Math.abs(velocityHeading.minus(snappedSquareEdge).getRadians());
    double improvementRad =
        Units.degreesToRadians(DriveConstants.EDGE_SNAP_HYSTERESIS_DEG);
    if (currentError - bestError < improvementRad) {
      return snappedSquareEdge;
    }

    snappedSquareEdge = bestEdge;
    return snappedSquareEdge;
  }

  /** Field-centric rotation PID toward {@code desiredFacing}. */
  public double calculateOmega(Rotation2d desiredFacing) {
    return fieldCentricAngleController.calculate(
        getRotation().getRadians(), desiredFacing.getRadians());
  }
}
