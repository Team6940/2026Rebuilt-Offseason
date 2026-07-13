package frc.robot;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.util.Color;
import java.io.IOException;
import java.nio.file.Path;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  /**
   * Publish AdvantageKit outputs to NetworkTables. On the roboRIO, the first write per topic can
   * block the robot loop while NT4 publishers are created; keep {@link #enableNtTelemetry} false
   * during competition if you only need USB wpilog. SIM may leave this true for AdvantageScope.
   */
  public static final boolean enableNtTelemetry = true;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static final class MotorIDs {

    /*   Chassis   */
    // Pigeon IMU
    public static final int kPigeonId = 0;
    // Front Left
    public static final int kFrontLeftDriveMotorId = 1;
    public static final int kFrontLeftSteerMotorId = 2;
    public static final int kFrontLeftEncoderId = 9;
    // Front Right
    public static final int kFrontRightDriveMotorId = 3;
    public static final int kFrontRightSteerMotorId = 4;
    public static final int kFrontRightEncoderId = 10;
    // Back Left
    public static final int kBackLeftDriveMotorId = 5;
    public static final int kBackLeftSteerMotorId = 6;
    public static final int kBackLeftEncoderId = 11;
    // Back Right
    public static final int kBackRightDriveMotorId = 7;
    public static final int kBackRightSteerMotorId = 8;
    public static final int kBackRightEncoderId = 12;

    /* Intake */
    public static final int kIntakeLeaderMotorId = 14;
    public static final int kIntakeFollowerMotorId = 15;
    public static final int kIntakeRackMotorId = 13;

    /* Shooter (two leader/follower pairs) */
    public static final int kShooterLeaderMotorIdA = 21;
    public static final int kShooterFollowerMotorIdA = 22;
    public static final int kShooterLeaderMotorIdB = 23;
    public static final int kShooterFollowerMotorIdB = 24;

    /* Hood */
    public static final int kHoodMotorId = 20;

    /* Indexer (two feeder + two indexer rollers) */
    public static final int kIndexerFeederLeaderMotorId = 18;
    public static final int kIndexerFeederFollowerMotorId = 19;
    public static final int kIndexerLeaderMotorId = 16;
    public static final int kIndexerFollowerMotorId = 17;
  }

  public static final class IntakeConstants {
    /* Rollers */
    public static final double RollerRatio = 35. / 20.;
    public static final InvertedValue RollerInverted = InvertedValue.Clockwise_Positive;
    public static final double RollerSupplyCurrentLimit = 20.0;
    public static final double RollerkP = 0.1;
    public static final double RollerkI = 0.0;
    public static final double RollerkD = 0.0;
    public static final double RollerkV = 0.213;
    public static final double RollerkS = 0.353;
    public static final double RollerVelocityToleranceRps = 0.5;
    public static final double IntakingRps = 40.0;
    public static final double IntakeEmergencyOutRps = 60.0;
    public static final MotorAlignmentValue RollerFollowerAlignment = MotorAlignmentValue.Opposed;

    /* Rack */
    public static final double RackRatio = 48. / 14.;
    public static final InvertedValue RackInverted = InvertedValue.Clockwise_Positive;
    public static final double RackSupplyCurrentLimit = 20.0;
    public static final double RackStatorCurrentLimit = 40.0;
    public static final double RackkP = 20.;
    public static final double RackkI = 0.0;
    public static final double RackkD = 0.0;
    public static final double RackkV = 0.0;
    public static final double RackkS = 0.34;
    public static final double RackMotionMagicMaxVelocity = 40.0;
    public static final double RackMotionMagicAcceleration = 80.0;
    public static final double RackPositionToleranceRotations = 0.01;
    public static final double RackMinRotations = 0.;
    public static final double RackMaxRotations = 4.59;
    public static final double RackIdleRotations = 0.;
    public static final double RackRetractedRotations = 2.41;
    public static final double RackMidRotations = 4.0;
    public static final double RackExtendedRotations = 4.58;

    /**
     * LT held longer than this sets {@link frc.robot.subsystems.SuperStructure.IntakeMode#HYBRID}.
     */
    public static final double LtHoldThresholdSec = 0.25;
  }

  public final class DriveConstants {
    public static final double Deadband = 0.05;

    /** Maximum seconds AttackMode stays active per enable. */
    public static final double AttackModeTimeoutSec = 10.0;

    public static final double AnglekP = 5.0;
    public static final double AnglekD = 0.4;
    public static final double AngleMaxVelocity = 8.0;
    public static final double AngleMaxAcceleration = 20.0;

    public static final double AutoAimAnglekP = 5.0;

    public static final double AutoAimAnglekI = 0.0;
    public static final double AutoAimAnglekD = 0.1;

    /**
     * Drive motor supply limit (A) while {@code driveAutoAim} is active; frees battery for shooter.
     */
    public static final double AutoAimDriveSupplyCurrentLimitAmps = 20.0;

    public static final double AutoAimDriveStatorCurrentLimitAmps = 40.0;

    /** Steer motor stator limit (A) while {@code driveAutoAim} is active. */
    public static final double AutoAimSteerSupplyCurrentLimitAmps = 25.0;

    public static final double AutoAimSteerStatorCurrentLimitAmps = 45.0;

    public static final double MoveToXkP = 5.;
    public static final double MoveToYkP = 5.;
    public static final double MoveToThetakP = 5.;
    public static final double MoveToXkD = 0.4;
    public static final double MoveToYkD = 0.4;
    public static final double MoveToThetakD = 0.4;
    public static final double MoveToPositionToleranceMeters = 0.05;
    public static final double MoveToAngleToleranceDegrees = 3.;

    public static final double PathPlannerTranslationkP = 5.;
    public static final double PathPlannerTranslationkD = 0.4;
    public static final double PathPlannerRotationkP = 5.;
    public static final double PathPlannerRotationkD = 0.4;

    /** Pure-pursuit-style lookahead distance along the trench centerline (m). */
    public static final double TrenchLookAheadMeters = 0.6;

    /** Field-relative driver speed (m/s) required to set path traversal direction. */
    public static final double TrenchDriveIntentThreshold = 0.08;

    /** Epsilon (m) for finite-difference path tangent at the nearest point. */
    public static final double TrenchTangentEpsilon = 0.05;

    /** Maximum fraction of max linear speed applied as trench guidance assist. */
    public static final double TrenchMaxAssist = 0.65;

    /** Minimum driver–guidance alignment (dot product) before assist ramps up. */
    public static final double TrenchMinAlignment = -0.2;

    /** Cross-track distance (m) at which path adhesion reaches zero. */
    public static final double TrenchMaxCrossTrackMeters = 1.0;

    /** Exponent on blended adhesion (0.5 = sqrt); boosts assist when near/on the path. */
    public static final double TrenchBlendExponent = 0.5;

    /** Degrees: new square edge must beat current edge by this much to switch. */
    public static final double EdgeSnapHysteresisDeg = 20.0;

    /** Trench heading hold. */
    public static final double TrenchAnglekP = 5.0;

    public static final double TrenchAnglekD = 0.1;
  }

  public final class OperatorConstants {
    /** Joystick deadband applied before scaling operator inputs. */
    public static final double DeadBand = 0.05;

    /**
     * Power exponent for the input curve. Values > 1 give finer control near center and bolder
     * response near full deflection. 2.0 = quadratic (recommended starting point).
     */
    public static final double InputPower = 2.0;
  }

  public static final class FieldConstants {
    private static final Path LIBRARY_LAYOUT_PATH =
        Filesystem.getDeployDirectory()
            .toPath()
            .resolve(Path.of("pathplanner", "field2026", "2026-official-andymark.json"));
    private static final AprilTagFieldLayout LAYOUT = loadAprilTagLayout();

    // AprilTag related constants
    public static final int aprilTagCount = LAYOUT.getTags().size();
    public static final double aprilTagWidth = Units.inchesToMeters(6.5);
    public static final double fieldLength = LAYOUT.getFieldLength();
    public static final double fieldWidth = LAYOUT.getFieldWidth();

    /**
     * Officially defined and relevant vertical lines found on the field (defined by X-axis offset)
     */
    public static class LinesVertical {
      public static final double center = fieldLength / 2.0;
      public static final double starting = getTagPoseOrDefault(26).getX();
      public static final double allianceZone = starting;
      public static final double hubCenter = getTagPoseOrDefault(26).getX() + Hub.width / 2.0;
      public static final double neutralZoneNear = center - Units.inchesToMeters(120);
      public static final double neutralZoneFar = center + Units.inchesToMeters(120);
      public static final double oppHubCenter = getTagPoseOrDefault(4).getX() + Hub.width / 2.0;
      public static final double oppAllianceZone = getTagPoseOrDefault(10).getX();
    }

    /**
     * Officially defined and relevant horizontal lines found on the field (defined by Y-axis
     * offset)
     *
     * <p>NOTE: The field element start and end are always left to right from the perspective of the
     * alliance station
     */
    public static class LinesHorizontal {

      public static final double center = fieldWidth / 2.0;

      // Right of hub
      public static final double rightBumpStart = Hub.nearRightCorner.getY();
      public static final double rightBumpEnd = rightBumpStart - RightBump.width;
      public static final double rightTrenchOpenStart = rightBumpEnd - Units.inchesToMeters(12.0);
      public static final double rightTrenchOpenEnd = 0;

      // Left of hub
      public static final double leftBumpEnd = Hub.nearLeftCorner.getY();
      public static final double leftBumpStart = leftBumpEnd + LeftBump.width;
      public static final double leftTrenchOpenEnd = leftBumpStart + Units.inchesToMeters(12.0);
      public static final double leftTrenchOpenStart = fieldWidth;
    }

    /** Hub related constants */
    public static class Hub {

      // Dimensions
      public static final double width = Units.inchesToMeters(47.0);
      public static final double height =
          Units.inchesToMeters(72.0); // includes the catcher at the top
      public static final double innerWidth = Units.inchesToMeters(41.7);
      public static final double innerHeight = Units.inchesToMeters(56.5);

      // Relevant reference points on alliance side
      public static final Translation3d topCenterPoint =
          new Translation3d(getTagPoseOrDefault(26).getX() + width / 2.0, fieldWidth / 2.0, height);
      public static final Translation3d innerCenterPoint =
          new Translation3d(
              getTagPoseOrDefault(26).getX() + width / 2.0, fieldWidth / 2.0, innerHeight);
      public static final Translation2d centerPoint = new Translation2d(4.623, 4.030); // 4.621 4.07

      public static final Translation2d nearLeftCorner =
          new Translation2d(topCenterPoint.getX() - width / 2.0, fieldWidth / 2.0 + width / 2.0);
      public static final Translation2d nearRightCorner =
          new Translation2d(topCenterPoint.getX() - width / 2.0, fieldWidth / 2.0 - width / 2.0);
      public static final Translation2d farLeftCorner =
          new Translation2d(topCenterPoint.getX() + width / 2.0, fieldWidth / 2.0 + width / 2.0);
      public static final Translation2d farRightCorner =
          new Translation2d(topCenterPoint.getX() + width / 2.0, fieldWidth / 2.0 - width / 2.0);

      // Relevant reference points on the opposite side
      public static final Translation3d oppTopCenterPoint =
          new Translation3d(getTagPoseOrDefault(4).getX() + width / 2.0, fieldWidth / 2.0, height);
      public static final Translation2d oppCenterPoint =
          new Translation2d(11.917, 4.030); // 11.9 4.07
      public static final Translation2d oppNearLeftCorner =
          new Translation2d(oppTopCenterPoint.getX() - width / 2.0, fieldWidth / 2.0 + width / 2.0);
      public static final Translation2d oppNearRightCorner =
          new Translation2d(oppTopCenterPoint.getX() - width / 2.0, fieldWidth / 2.0 - width / 2.0);
      public static final Translation2d oppFarLeftCorner =
          new Translation2d(oppTopCenterPoint.getX() + width / 2.0, fieldWidth / 2.0 + width / 2.0);
      public static final Translation2d oppFarRightCorner =
          new Translation2d(oppTopCenterPoint.getX() + width / 2.0, fieldWidth / 2.0 - width / 2.0);

      // Hub faces
      public static final Pose2d nearFace = getTagPoseOrDefault(26).toPose2d();
      public static final Pose2d farFace = getTagPoseOrDefault(20).toPose2d();
      public static final Pose2d rightFace = getTagPoseOrDefault(18).toPose2d();
      public static final Pose2d leftFace = getTagPoseOrDefault(21).toPose2d();
    }

    /** Left Bump related constants */
    public static class LeftBump {

      // Dimensions
      public static final double width = Units.inchesToMeters(73.0);
      public static final double height = Units.inchesToMeters(6.513);
      public static final double depth = Units.inchesToMeters(44.4);

      // Relevant reference points on alliance side
      public static final Translation2d nearLeftCorner =
          new Translation2d(LinesVertical.hubCenter - width / 2, Units.inchesToMeters(255));
      public static final Translation2d nearRightCorner = Hub.nearLeftCorner;
      public static final Translation2d farLeftCorner =
          new Translation2d(LinesVertical.hubCenter + width / 2, Units.inchesToMeters(255));
      public static final Translation2d farRightCorner = Hub.farLeftCorner;

      // Relevant reference points on opposing side
      public static final Translation2d oppNearLeftCorner =
          new Translation2d(LinesVertical.hubCenter - width / 2, Units.inchesToMeters(255));
      public static final Translation2d oppNearRightCorner = Hub.oppNearLeftCorner;
      public static final Translation2d oppFarLeftCorner =
          new Translation2d(LinesVertical.hubCenter + width / 2, Units.inchesToMeters(255));
      public static final Translation2d oppFarRightCorner = Hub.oppFarLeftCorner;
    }

    /** Right Bump related constants */
    public static class RightBump {
      // Dimensions
      public static final double width = Units.inchesToMeters(73.0);
      public static final double height = Units.inchesToMeters(6.513);
      public static final double depth = Units.inchesToMeters(44.4);

      // Relevant reference points on alliance side
      public static final Translation2d nearLeftCorner =
          new Translation2d(LinesVertical.hubCenter + width / 2, Units.inchesToMeters(255));
      public static final Translation2d nearRightCorner = Hub.nearLeftCorner;
      public static final Translation2d farLeftCorner =
          new Translation2d(LinesVertical.hubCenter - width / 2, Units.inchesToMeters(255));
      public static final Translation2d farRightCorner = Hub.farLeftCorner;

      // Relevant reference points on opposing side
      public static final Translation2d oppNearLeftCorner =
          new Translation2d(LinesVertical.hubCenter + width / 2, Units.inchesToMeters(255));
      public static final Translation2d oppNearRightCorner = Hub.oppNearLeftCorner;
      public static final Translation2d oppFarLeftCorner =
          new Translation2d(LinesVertical.hubCenter - width / 2, Units.inchesToMeters(255));
      public static final Translation2d oppFarRightCorner = Hub.oppFarLeftCorner;
    }

    /** Left Trench related constants */
    public static class LeftTrench {
      // Dimensions
      public static final double width = Units.inchesToMeters(65.65);
      public static final double depth = Units.inchesToMeters(47.0);
      public static final double height = Units.inchesToMeters(40.25);
      public static final double openingWidth = Units.inchesToMeters(50.34);
      public static final double openingHeight = Units.inchesToMeters(22.25);

      // Relevant reference points on alliance side
      public static final Translation3d openingTopLeft =
          new Translation3d(LinesVertical.hubCenter, fieldWidth, openingHeight);
      public static final Translation3d openingTopRight =
          new Translation3d(LinesVertical.hubCenter, fieldWidth - openingWidth, openingHeight);

      // Relevant reference points on opposing side
      public static final Translation3d oppOpeningTopLeft =
          new Translation3d(LinesVertical.oppHubCenter, fieldWidth, openingHeight);
      public static final Translation3d oppOpeningTopRight =
          new Translation3d(LinesVertical.oppHubCenter, fieldWidth - openingWidth, openingHeight);
    }

    public static class RightTrench {

      // Dimensions
      public static final double width = Units.inchesToMeters(65.65);
      public static final double depth = Units.inchesToMeters(47.0);
      public static final double height = Units.inchesToMeters(40.25);
      public static final double openingWidth = Units.inchesToMeters(50.34);
      public static final double openingHeight = Units.inchesToMeters(22.25);

      // Relevant reference points on alliance side
      public static final Translation3d openingTopLeft =
          new Translation3d(LinesVertical.hubCenter, openingWidth, openingHeight);
      public static final Translation3d openingTopRight =
          new Translation3d(LinesVertical.hubCenter, 0, openingHeight);

      // Relevant reference points on opposing side
      public static final Translation3d oppOpeningTopLeft =
          new Translation3d(LinesVertical.oppHubCenter, openingWidth, openingHeight);
      public static final Translation3d oppOpeningTopRight =
          new Translation3d(LinesVertical.oppHubCenter, 0, openingHeight);
    }

    /** Tower related constants */
    public static class Tower {
      // Dimensions
      public static final double width = Units.inchesToMeters(49.25);
      public static final double depth = Units.inchesToMeters(45.0);
      public static final double height = Units.inchesToMeters(78.25);
      public static final double innerOpeningWidth = Units.inchesToMeters(32.250);
      public static final double frontFaceX = Units.inchesToMeters(43.51);

      public static final double uprightHeight = Units.inchesToMeters(72.1);

      // Rung heights from the floor
      public static final double lowRungHeight = Units.inchesToMeters(27.0);
      public static final double midRungHeight = Units.inchesToMeters(45.0);
      public static final double highRungHeight = Units.inchesToMeters(63.0);

      // Relevant reference points on alliance side
      public static final Translation2d centerPoint =
          new Translation2d(frontFaceX, getTagPoseOrDefault(31).getY());
      public static final Translation2d leftUpright =
          new Translation2d(
              frontFaceX,
              (getTagPoseOrDefault(31).getY())
                  + innerOpeningWidth / 2
                  + Units.inchesToMeters(0.75));
      public static final Translation2d rightUpright =
          new Translation2d(
              frontFaceX,
              (getTagPoseOrDefault(31).getY())
                  - innerOpeningWidth / 2
                  - Units.inchesToMeters(0.75));

      // Relevant reference points on opposing side
      public static final Translation2d oppCenterPoint =
          new Translation2d(fieldLength - frontFaceX, getTagPoseOrDefault(15).getY());
      public static final Translation2d oppLeftUpright =
          new Translation2d(
              fieldLength - frontFaceX,
              (getTagPoseOrDefault(15).getY())
                  + innerOpeningWidth / 2
                  + Units.inchesToMeters(0.75));
      public static final Translation2d oppRightUpright =
          new Translation2d(
              fieldLength - frontFaceX,
              (getTagPoseOrDefault(15).getY())
                  - innerOpeningWidth / 2
                  - Units.inchesToMeters(0.75));
    }

    public static class Depot {
      // Dimensions
      public static final double width = Units.inchesToMeters(42.0);
      public static final double depth = Units.inchesToMeters(27.0);
      public static final double height = Units.inchesToMeters(1.125);
      public static final double distanceFromCenterY = Units.inchesToMeters(75.93);

      // Relevant reference points on alliance side
      public static final Translation3d depotCenter =
          new Translation3d(depth, (fieldWidth / 2) + distanceFromCenterY, height);
      public static final Translation3d leftCorner =
          new Translation3d(depth, (fieldWidth / 2) + distanceFromCenterY + (width / 2), height);
      public static final Translation3d rightCorner =
          new Translation3d(depth, (fieldWidth / 2) + distanceFromCenterY - (width / 2), height);
    }

    public static class Outpost {
      // Dimensions
      public static final double width = Units.inchesToMeters(31.8);
      public static final double openingDistanceFromFloor = Units.inchesToMeters(28.1);
      public static final double height = Units.inchesToMeters(7.0);

      // Relevant reference points on alliance side
      public static final Translation2d centerPoint =
          new Translation2d(0, getTagPoseOrDefault(29).getY());
    }

    private static Pose3d getTagPoseOrDefault(int id) {
      return LAYOUT
          .getTagPose(id)
          .orElseGet(
              () -> {
                DriverStation.reportError(
                    "Missing AprilTag ID " + id + " in layout: " + LIBRARY_LAYOUT_PATH, false);
                return new Pose3d();
              });
    }

    /** WPILib field layout for PhotonVision pose estimation and tag lookups. */
    public static AprilTagFieldLayout getAprilTagFieldLayout() {
      return LAYOUT;
    }

    private static AprilTagFieldLayout loadAprilTagLayout() {
      try {
        return new AprilTagFieldLayout(LIBRARY_LAYOUT_PATH);
      } catch (IOException e) {
        DriverStation.reportError(
            "Failed to load AprilTag layout from " + LIBRARY_LAYOUT_PATH, e.getStackTrace());
        return new AprilTagFieldLayout(new java.util.ArrayList<>(), 0.0, 0.0);
      }
    }
  }

  public static final class ProjectileConstants {
    /** Distance (m) -> shooter RPS for static (SCORE) shots. */
    public static final InterpolatingDoubleTreeMap DistanceToShooterRps =
        new InterpolatingDoubleTreeMap();

    /** Distance (m) -> hood angle (deg) for static (SCORE) shots. */
    public static final InterpolatingDoubleTreeMap DistanceToHoodDegs =
        new InterpolatingDoubleTreeMap();

    /** Distance (m) -> flight time (s); used by PASS motion solver only. */
    public static final InterpolatingDoubleTreeMap DistanceToFlightTimeSecs =
        new InterpolatingDoubleTreeMap();

    /** Distance (m) -> shooter RPS for PASS motion solver lookups. */
    public static final InterpolatingDoubleTreeMap PassDistanceToShooterRps =
        new InterpolatingDoubleTreeMap();

    /** Distance (m) -> hood angle (deg) for PASS motion solver lookups. */
    public static final InterpolatingDoubleTreeMap PassDistanceToHoodDegs =
        new InterpolatingDoubleTreeMap();

    static {
      DistanceToShooterRps.put(0.947, 27.9);
      DistanceToShooterRps.put(1.32, 29.0);
      DistanceToShooterRps.put(1.88, 34.04);
      DistanceToShooterRps.put(2.1, 35.54);
      DistanceToShooterRps.put(2.6, 36.2);
      DistanceToShooterRps.put(3.4, 39.4);
      DistanceToShooterRps.put(3.6, 41.4);
      DistanceToShooterRps.put(4.1, 42.4);
      DistanceToShooterRps.put(4.99, 46.9);
      DistanceToShooterRps.put(5.2, 51.5);
      DistanceToShooterRps.put(5.7, 53.5);

      DistanceToHoodDegs.put(0.947, 17.842);
      DistanceToHoodDegs.put(1.32, 17.842);
      DistanceToHoodDegs.put(1.88, 20.40);
      DistanceToHoodDegs.put(2.6, 21.0);
      DistanceToHoodDegs.put(3.4, 23.3);
      DistanceToHoodDegs.put(4.1, 26.5);
      DistanceToHoodDegs.put(4.99, 30.5);
      DistanceToHoodDegs.put(5.2, 38.5);
      DistanceToShooterRps.put(5.7, 41.);

      DistanceToFlightTimeSecs.put(0.96, 0.8);
      DistanceToFlightTimeSecs.put(1.2, 0.95);
      DistanceToFlightTimeSecs.put(3.0, 1.18);
      DistanceToFlightTimeSecs.put(5.0, 1.28);

      PassDistanceToShooterRps.put(1.05, 14.9);
      PassDistanceToShooterRps.put(1.32, 19.61);
      PassDistanceToShooterRps.put(1.88, 21.84);
      PassDistanceToShooterRps.put(2.6, 23.8);
      PassDistanceToShooterRps.put(3.4, 27.4);
      PassDistanceToShooterRps.put(4.1, 31.4);
      PassDistanceToShooterRps.put(4.99, 34.9);
      PassDistanceToShooterRps.put(6.0, 37.5);
      PassDistanceToShooterRps.put(7.0, 54.5);
      PassDistanceToShooterRps.put(13.0, 54.);

      PassDistanceToHoodDegs.put(1.05, 17.842);
      PassDistanceToHoodDegs.put(1.32, 17.842);
      PassDistanceToHoodDegs.put(1.88, 19.40);
      PassDistanceToHoodDegs.put(2.6, 23.3);
      PassDistanceToHoodDegs.put(3.4, 29.9);
      PassDistanceToHoodDegs.put(4.1, 34.5);
      PassDistanceToHoodDegs.put(4.99, 38.5);
      PassDistanceToHoodDegs.put(5.2, 40.5);
      PassDistanceToHoodDegs.put(7., 40.5);
      PassDistanceToHoodDegs.put(10., 43.5);
    }
  }

  public static final class ShooterConstants {
    public static final double ShooterRatio = 1.5;
    public static final InvertedValue AInverted = InvertedValue.Clockwise_Positive;
    public static final InvertedValue BInverted = InvertedValue.CounterClockwise_Positive;
    public static final double VelocityToleranceRps = 0.;
    public static final MotorAlignmentValue FollowerAlignment = MotorAlignmentValue.Aligned;
    public static final double kP = 8.;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    public static final double kV = 0.11;
    public static final double kS = 9;
    public static final double SupplyCurrentLimit = 60.0;
    public static final double StatorCurrentLimit = 60.0;

    /** Robot origin to shooter exit point (+X forward, +Y left), meters. */
    public static final Translation2d ShooterOffset = new Translation2d(-0.24, 0.0);

    /** Sim velocity lag (s): lower = faster spin-up, higher = smoother. */
    public static final double SimSpinupTimeConstantSec = 0.35;
  }

  public static final class HoodConstants {
    public static final double HoodRatio = 31. / 20. * 40. / 14. * 14.;
    public static final InvertedValue Inverted = InvertedValue.Clockwise_Positive;
    public static final double SupplyCurrentLimit = 40.0;
    public static final double kP = 70.0;
    public static final double kI = 0.0;
    public static final double kD = 0.2;
    public static final double kV = 0.0;
    public static final double kS = 0.0;
    public static final double kG = 0.4;
    public static final double MotionMagicMaxVelocity = 40.0;
    public static final double MotionMagicAcceleration = 80.0;
    public static final double PositionToleranceDegs = 0.0;
    public static final double MinDegs = 17.842;
    public static final double MaxDegs = 57.6;
    public static final double IdlePositionDegs = 17.842;
  }

  /** Tuning for hybrid shoot commands ({@link frc.robot.commands.HybridScoreCommand}). */
  public static final class HybridShootConstants {
    // --- Ready gate (AIM to READY) ---
    public static final double HeadingToleranceDegs = 1.5;
    public static final double HoodToleranceDegs = 1.5;
    public static final double ShooterToleranceRpsLower = 3.;
    public static final double ShooterToleranceRpsHigher = 2.;

    // --- Operator trims (AIM / READY) ---
    public static final double AimHeadingCompRangeDegs = 10.0;
    public static final double HoodCompRangeDegs = 3.0;

    // --- SHOOT: locked drive fine-tune (operator right stick) ---
    public static final double ShootHeadingFineTuneDeadband = 0.3;
    public static final double DriverTranslationFineTuneDeadband = 0.3;

    // --- SHOOT: indexer feed, then intake retract timing ---
    public static final double FeedDurationSec = 0.6;
    public static final double PostRetractWaitSec = 0.4;

    // --- Operator RPS offset steps (B / A / X / Y) ---
    public static final double RpsOffsetB = -1.0;
    public static final double RpsOffsetA = -2.0;
    public static final double RpsOffsetX = 1.0;
    public static final double RpsOffsetY = 2.0;
  }

  public static final class IndexerConstants {
    public static final double FeederRatio = 1.7;
    public static final double IndexerRatio = 1.0;
    public static final InvertedValue FeederInverted = InvertedValue.Clockwise_Positive;
    public static final InvertedValue IndexerInverted = InvertedValue.Clockwise_Positive;
    public static final double FeederSupplyCurrentLimit = 30.0;
    public static final double IndexerSupplyCurrentLimit = 20.0;
    public static final double FeederkP = 0.1;
    public static final double FeederkI = 0.0;
    public static final double FeederkD = 0.0;
    public static final double FeederkV = 0.12;
    public static final double FeederkS = 0.31;
    public static final double IndexerkP = 0.1;
    public static final double IndexerkI = 0.0;
    public static final double IndexerkD = 0.0;
    public static final double IndexerkV = 0.12;
    public static final double IndexerkS = 0.325;
    public static final MotorAlignmentValue IndexerFollowerAlignment = MotorAlignmentValue.Opposed;
    public static final MotorAlignmentValue FeederFollowerAlignment = MotorAlignmentValue.Aligned;
    public static final double FeedRps = 75.0;
    public static final double IndexerRps = 60.0;
  }

  /** Maple-sim field sim tuning for OverTheBumper intake, hopper, and full-width dumper shooter. */
  public static final class FieldSimulationConstants {
    public static final String FUEL_TYPE = "Fuel";

    /** Over-the-bumper intake mounted on the front of the chassis. */
    public static final double OverTheBumperIntakeWidthMeters = 0.85;

    /** Rack extended length beyond bumper (m). */
    public static final double OverTheBumperIntakeExtensionMeters = 0.25;

    /** Simulated hopper / intake storage capacity (Fuel count). */
    public static final int HopperCapacity = 50;

    /** Full-width rear dumper: parallel fuel count per volley (left / center / right). */
    public static final int DumperFuelPerVolley = 3;

    /** Lateral spacing between parallel dump lanes in robot frame (+Y left), meters. */
    public static final double DumperLateralSpacingMeters = 0.18;

    /** Minimum time between full-width volleys while feeding (s). */
    public static final double DumperVolleyPeriodSec = 0.10;

    /** Full-width rear dumper: exit height above carpet (m). */
    public static final double DumperExitHeightMeters = 0.52;

    /** Full-width dumper: m/s per shooter mechanism RPS (lower than flywheel). */
    public static final double DumperMetersPerSecondPerRps = 0.055 * 4;

    public static final double DumperMinLaunchSpeedMps = 2.5;

    /** Hub goal center height for hit detection (m). Matches Rebuilt hub opening region. */
    public static final double HubTargetHeightMeters = 1.35;

    /** Scoring tolerance (m): X, Y full-width, Z vertical. */
    public static final double HubTargetToleranceXMeters = 0.55;

    public static final double HubTargetToleranceYMeters = 1.25;
    public static final double HubTargetToleranceZMeters = 0.35;
  }

  /**
   * Vision fusion: shared rejection gates, Photon yaw scaling, and Limelight-independent constants.
   */
  public static final class VisionFusion {
    /** AprilTag IDs on alliance / opponent hub faces (z = 1.12+-0.2 m on 2026 field). */
    public static final int[] HUB_TAG_IDS = {
      2, 3, 4, 5, 8, 9, 10, 11, 18, 19, 20, 21, 24, 25, 26, 27
    };

    public static final int[] TRENCH_TAG_IDS = {7, 6, 12, 1, 17, 28, 22, 23};

    public static boolean isHubTag(int tagId) {
      for (int hubId : HUB_TAG_IDS) {
        if (hubId == tagId) {
          return true;
        }
      }
      return false;
    }

    public static boolean isTrenchTag(int tagId) {
      for (int trenchId : TRENCH_TAG_IDS) {
        if (trenchId == tagId) {
          return true;
        }
      }
      return false;
    }

    /** Minimum divisor when penalizing large horizontal targeting angles via cos(yaw). */
    public static final double REJECT_MIN_TA = 0.01;

    /**
     * Reject when estimated tag distance to robot exceeds this (meters). Tune per camera mounting.
     */
    public static final double REJECT_MAX_DISTANCE_METERS = 4.0;

    public static final double REJECT_STALE_SECONDS = 0.5;
    public static final double REJECT_MAX_OMEGA_RAD_PER_SEC = 4.0 * Math.PI;
    public static final boolean REJECT_ON_HIGH_OMEGA = true;

    /**
     * Robot origin to PhotonBack AprilTag camera (WPILib: +X forward, +Y left, +Z up). Mount at
     * (6.46 mm, -342 mm, 501 mm) relative to robot geometric center, facing backward (-X).
     */
    public static final Transform3d kRobotToPhotonBack =
        new Transform3d(
            new Translation3d(-0.342, 0., 0.501), new Rotation3d(Math.PI / 12., 0.0, Math.PI));

    /**
     * Pose θ standard deviation (rad) for Photon when xy σ comes from {@link
     * PoseEstimatorConstants#tAtoDev} - very large so fusion weights gyro for heading.
     */
    public static final double PHOTON_THETA_STDDEV_RADIANS = 100000000.0;

    /** Down-weights unreliable heading during multi-source inverse-variance fusion. */
    public static final double LARGE_VARIANCE = 1e6;

    /** Limelight NT {@code stddevs} array length and MegaTag index offsets. */
    public static final int LL_STDDEV_ARRAY_LENGTH = 12;

    public static final int LL_MT1_X_STDDEV_INDEX = 0;
    public static final int LL_MT1_Y_STDDEV_INDEX = 1;
    public static final int LL_MT1_YAW_STDDEV_INDEX = 5;
    public static final int LL_MT2_X_STDDEV_INDEX = 6;
    public static final int LL_MT2_Y_STDDEV_INDEX = 7;
    public static final int LL_MT2_YAW_STDDEV_INDEX = 11;
  }

  /**
   * Interpolates vision translation sigma (m) from target area fraction (0–1). Used by Photon;
   * Limelight MegaTag2 uses hardware {@code stddevs} instead.
   */
  public static final class PoseEstimatorConstants {
    public static final InterpolatingDoubleTreeMap tAtoDev = new InterpolatingDoubleTreeMap();

    static {
      tAtoDev.put(0.17, 0.08);
      tAtoDev.put(0.12, 0.20);
      tAtoDev.put(0.071, 0.35);
      tAtoDev.put(0.046, 0.4);
      tAtoDev.put(0.03, 0.7);
      tAtoDev.put(0.01, 1.0);
    }
  }

  public static class Ports {
    public static class LED {
      public static final int LEDPWMPort = 8;
    }
  }

  public static class Settings {
    public static class LED {
      public static final int LEDLength = 35;
      public static final int[] GyroBuffer = {};
      public static final int[] ChassisLeft = {0, 4};
      public static final int[] ShooterMid = {4, 27};
      public static final int[] ChassisRight = {30, 34};
      public static final Color AttackModeColor = new Color("#267ce4");
    }

    public interface LEDs {

      // TODO: Get actual length of led, along with length of individual sections
      int LED_LENGTH = Settings.LED.LEDLength;
      // LED Pattern

      LEDPattern DISABLED = LEDPattern.solid(Color.kPurple);

      LEDPattern CHASSIS_NORMAL = LEDPattern.solid(Color.kWhite);
      LEDPattern CHASSIS_SHOOTING = LEDPattern.solid(Color.kRed);
      LEDPattern CHASSIS_ATTACKMODE = LEDPattern.solid(Settings.LED.AttackModeColor);
      LEDPattern MANUAL = LEDPattern.solid(Color.kPurple);
      LEDPattern SHOOT = LEDPattern.solid(Color.kRed);
      LEDPattern READY = LEDPattern.solid(Color.kGreen);
      LEDPattern AUTO_AIM = LEDPattern.solid(Color.kYellow);
      LEDPattern AIM = LEDPattern.solid(Color.kYellow);
      LEDPattern HEATUP = LEDPattern.solid(Color.kWhite);
      LEDPattern OFF = LEDPattern.solid(Color.kAliceBlue);
    }
  }
}
