package frc.robot.autos;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.AutoConstants;
import frc.robot.lib.BLine.FlippingUtil;
import frc.robot.lib.BLine.FollowPath;
import frc.robot.lib.BLine.Path;
import frc.robot.lib.BLine.Path.PathElement;
import frc.robot.lib.BLine.Path.PathElementConstraint;
import frc.robot.subsystems.Chassis.CommandSwerveDrivetrain;
import java.util.ArrayList;
import java.util.List;

/** BLine auto utilities ported from Spartronics 4915. */
public final class Autos {
  private Autos() {}

  public static FollowPath.Builder pathBuilder;

  public static boolean surveyMode = false;
  public static final Field2d surveyField = new Field2d();
  private static final List<PathElement> surveyElements = new ArrayList<>();
  private static final List<Pose2d> surveyPoses = new ArrayList<>();
  private static final List<Pose2d> surveyPathEndPoses = new ArrayList<>();
  private static final List<Integer> rawPathEndIndices = new ArrayList<>();
  public static Pose2d swervePose = new Pose2d();
  public static double surveyPathProgressMeters = 0.0;

  private static Command currentSurveyCommand = null;

  static {
    SmartDashboard.putData("Auto Chooser/Survey Field", surveyField);
  }

  public static void setPathBuilder(FollowPath.Builder builder) {
    pathBuilder = builder;
  }

  public static Command wait(double seconds) {
    return surveyMode ? Commands.none() : Commands.waitSeconds(seconds);
  }

  public static void setSwervePose(Pose2d pose) {
    swervePose = pose;
    if (surveyMode) {
      displaySurveyField();
      surveyPathProgressMeters += 0.02 * 3.0;
    }
  }

  public static void survey(Command autoCommand) {
    if (currentSurveyCommand != null) {
      currentSurveyCommand.cancel();
    }
    surveyMode = true;
    surveyPathProgressMeters = 0.0;
    surveyElements.clear();
    surveyPoses.clear();
    surveyPathEndPoses.clear();
    rawPathEndIndices.clear();

    currentSurveyCommand =
        Commands.sequence(
                autoCommand,
                Commands.runOnce(
                    () -> {
                      processElements();
                      displaySurveyField();
                      surveyMode = false;
                    }))
            .ignoringDisable(true)
            .withDeadline(Commands.waitUntil(() -> !surveyMode));

    CommandScheduler.getInstance().schedule(currentSurveyCommand);
  }

  public static Command build(Path path) {
    return build(path, null, null);
  }

  public static Command build(
      Path path, Rotation2d endWithSpeedDirection, CommandSwerveDrivetrain drive) {
    if (surveyMode) {
      surveyElements.addAll(path.getPathElements());
      rawPathEndIndices.add(surveyElements.size() - 1);
      return Commands.none();
    }

    Translation2d overshootTarget = null;
    Path followPath = path;

    if (endWithSpeedDirection != null) {
      List<Pair<PathElement, PathElementConstraint>> waypoints =
          path.getPathElementsWithConstraintsNoWaypoints();
      Translation2d finalWaypoint = null;

      for (int i = waypoints.size() - 1; i >= 0; i--) {
        PathElement element = waypoints.get(i).getFirst();
        if (element instanceof Path.RotationTarget) {
          continue;
        }

        Translation2d translation = null;
        if (element instanceof Path.TranslationTarget translationTarget) {
          translation = translationTarget.translation();
        } else if (element instanceof Path.Waypoint waypoint) {
          translation = waypoint.translationTarget().translation();
        }

        if (translation != null) {
          finalWaypoint = translation;
          break;
        }
      }

      if (finalWaypoint != null) {
        overshootTarget =
            new Translation2d(AutoConstants.velocityEndingDistance.in(Meters), 0)
                .rotateBy(endWithSpeedDirection)
                .plus(finalWaypoint);
        followPath = copyPath(path);
        followPath.addPathElement(new Path.TranslationTarget(overshootTarget));
      }
    }

    Command pathCommand = pathBuilder.build(followPath);

    if (drive != null && overshootTarget != null) {
      final Translation2d overshoot = overshootTarget;
      return Commands.race(
          pathCommand,
          Commands.waitUntil(
              () ->
                  drive
                          .getRelativePose()
                          .getTranslation()
                          .minus(overshoot)
                          .getNorm()
                      <= AutoConstants.velocityEndingDistance.in(Meters)));
    }

    return pathCommand;
  }

  private static Path copyPath(Path source) {
    List<PathElement> elements = new ArrayList<>(source.getPathElements());
    Path.PathConstraints constraints = source.getPathConstraints();
    if (constraints != null) {
      return new Path(elements, constraints);
    }
    return new Path(elements);
  }

  public static void removePastPoses(
      CommandSwerveDrivetrain drive, List<PathElement> waypoints, boolean toNeutralZone) {
    if (surveyMode || !toNeutralZone) {
      return;
    }

    double x = drive.getRelativePose().getX();

    for (int i = waypoints.size() - 1; i >= 0; i--) {
      PathElement element = waypoints.get(i);

      if (element instanceof Path.RotationTarget) {
        continue;
      }

      Pose2d relativePose = drive.getRelativePose();

      if (element instanceof Path.TranslationTarget translationTarget) {
        if ((translationTarget.translation().getX() > x) ^ toNeutralZone) {
          waypoints.remove(i);
          waypoints.add(i, new Path.TranslationTarget(relativePose.getTranslation()));
        }
      }

      if (element instanceof Path.Waypoint waypoint) {
        if ((waypoint.translationTarget().translation().getX() > x) ^ toNeutralZone) {
          waypoints.remove(i);
          waypoints.add(i, new Path.Waypoint(relativePose));
        }
      }
    }
  }

  public static Command generatePathFromWaypoint(
      CommandSwerveDrivetrain drive, Translation2d translation, Rotation2d endingHeading) {
    return generatePathFromWaypoint(drive, translation, endingHeading, null);
  }

  public static Command generatePathFromWaypoint(
      CommandSwerveDrivetrain drive,
      Translation2d translation,
      Rotation2d endingHeading,
      Path.PathConstraints pathConstraints) {
    Pose2d waypoint = new Pose2d(translation, endingHeading);
    List<PathElement> pathElements = new ArrayList<>(List.of(new Path.Waypoint(waypoint)));

    Path path =
        pathConstraints != null ? new Path(pathElements, pathConstraints) : new Path(pathElements);
    return build(path);
  }

  public static Path.PathConstraints generatePathConstraintZone(
      Path.PathConstraints constraints, int start, int end) {
    Path.PathConstraints limited = new Path.PathConstraints();

    constraints
        .getMaxVelocityMetersPerSec()
        .ifPresent(
            values ->
                limited.setMaxVelocityMetersPerSec(
                    new Path.RangedConstraint(values.get(0).value(), start, end)));
    constraints
        .getMaxAccelerationMetersPerSec2()
        .ifPresent(
            values ->
                limited.setMaxAccelerationMetersPerSec2(
                    new Path.RangedConstraint(values.get(0).value(), start, end)));
    constraints
        .getMaxVelocityDegPerSec()
        .ifPresent(
            values ->
                limited.setMaxVelocityDegPerSec(
                    new Path.RangedConstraint(values.get(0).value(), start, end)));
    constraints
        .getMaxAccelerationDegPerSec2()
        .ifPresent(
            values ->
                limited.setMaxAccelerationDegPerSec2(
                    new Path.RangedConstraint(values.get(0).value(), start, end)));

    return limited;
  }

  public static Path.PathConstraints combineConstraints(Path.PathConstraints... constraintsList) {
    Path.PathConstraints combined = new Path.PathConstraints();
    List<Path.RangedConstraint> vel = new ArrayList<>();
    List<Path.RangedConstraint> acc = new ArrayList<>();
    List<Path.RangedConstraint> velDeg = new ArrayList<>();
    List<Path.RangedConstraint> accDeg = new ArrayList<>();

    for (Path.PathConstraints constraints : constraintsList) {
      constraints.getMaxVelocityMetersPerSec().ifPresent(vel::addAll);
      constraints.getMaxAccelerationMetersPerSec2().ifPresent(acc::addAll);
      constraints.getMaxVelocityDegPerSec().ifPresent(velDeg::addAll);
      constraints.getMaxAccelerationDegPerSec2().ifPresent(accDeg::addAll);
    }

    if (!vel.isEmpty()) {
      combined =
          combined.setMaxVelocityMetersPerSec(vel.toArray(new Path.RangedConstraint[0]));
    }
    if (!acc.isEmpty()) {
      combined =
          combined.setMaxAccelerationMetersPerSec2(acc.toArray(new Path.RangedConstraint[0]));
    }
    if (!velDeg.isEmpty()) {
      combined = combined.setMaxVelocityDegPerSec(velDeg.toArray(new Path.RangedConstraint[0]));
    }
    if (!accDeg.isEmpty()) {
      combined =
          combined.setMaxAccelerationDegPerSec2(accDeg.toArray(new Path.RangedConstraint[0]));
    }

    return combined;
  }

  public static Pose2d flipIfNeeded(Pose2d pose) {
    if (shouldFlip()) {
      return FlippingUtil.flipFieldPose(pose);
    }
    return pose;
  }

  public static boolean shouldFlip() {
    return DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;
  }

  public static void resetAllianceCache() {
    // Kept for API compatibility; alliance is read live from DriverStation.
  }

  private static void processElements() {
    Rotation2d lastRotation = Rotation2d.kZero;

    record SurveyRotationTarget(double index, Rotation2d rotation) {}
    List<SurveyRotationTarget> rotationTargets = new ArrayList<>();

    for (int i = 0; i < surveyElements.size(); i++) {
      PathElement element = surveyElements.get(i);
      if (element instanceof Path.Waypoint waypoint) {
        surveyPoses.add(
            new Pose2d(
                waypoint.translationTarget().translation(), waypoint.rotationTarget().rotation()));
        lastRotation = waypoint.rotationTarget().rotation();
      } else if (element instanceof Path.RotationTarget rotationTarget) {
        rotationTargets.add(
            new SurveyRotationTarget(
                surveyPoses.size() - 1 + rotationTarget.t_ratio(), rotationTarget.rotation()));
        lastRotation = rotationTarget.rotation();
      } else if (element instanceof Path.TranslationTarget translationTarget) {
        surveyPoses.add(new Pose2d(translationTarget.translation(), lastRotation));
      }

      if (rawPathEndIndices.contains(i)) {
        surveyPathEndPoses.add(surveyPoses.get(Math.max(0, surveyPoses.size() - 1)));
      }
    }

    for (int i = surveyPoses.size() - 1; i >= 0; i--) {
      Pose2d pose = surveyPoses.get(i);
      Pose2d flipped = flipIfNeeded(pose);
      if (flipped.relativeTo(swervePose).getTranslation().getNorm() <= 0.1) {
        surveyPoses.remove(i);
        for (int j = rotationTargets.size() - 1; j >= 0; j--) {
          if (rotationTargets.get(j).index() < i) {
            break;
          }
          rotationTargets.set(
              j,
              new SurveyRotationTarget(
                  rotationTargets.get(j).index() - 1, rotationTargets.get(j).rotation()));
        }
      }
    }

    for (int i = rotationTargets.size() - 1; i >= 0; i--) {
      double targetIndex = rotationTargets.get(i).index();
      Rotation2d rotation = rotationTargets.get(i).rotation();
      int floorIndex = (int) Math.floor(targetIndex);
      int ceilIndex = (int) Math.ceil(targetIndex);
      if (floorIndex < 0 || floorIndex >= surveyPoses.size() || ceilIndex >= surveyPoses.size()) {
        continue;
      }
      Translation2d floorTranslation = surveyPoses.get(floorIndex).getTranslation();
      Translation2d ceilTranslation = surveyPoses.get(ceilIndex).getTranslation();
      double interpolationRatio = targetIndex - floorIndex;
      Translation2d interpolatedTranslation =
          floorTranslation.interpolate(ceilTranslation, interpolationRatio);
      surveyPoses.add(floorIndex + 1, new Pose2d(interpolatedTranslation, rotation));
    }
  }

  private static void displaySurveyField() {
    List<Pose2d> finalWaypoints = new ArrayList<>();
    for (int i = 0; i < 10; i++) {
      finalWaypoints.add(swervePose);
    }
    for (Pose2d waypoint : surveyPoses) {
      finalWaypoints.add(flipIfNeeded(waypoint));
    }

    surveyField.getObject("waypoints").setPoses(finalWaypoints);

    for (int i = 0; i < surveyPathEndPoses.size(); i++) {
      surveyField.getObject("pathEnd" + i).setPose(flipIfNeeded(surveyPathEndPoses.get(i)));
    }
    for (int i = surveyPathEndPoses.size(); i < 20; i++) {
      surveyField.getObject("pathEnd" + i).setPoses(new Pose2d[0]);
    }

    surveyField.setRobotPose(swervePose);

    double surveyPathMetersLeft = surveyPathProgressMeters;
    for (int i = 1; i < finalWaypoints.size(); i++) {
      double dist =
          finalWaypoints.get(i).getTranslation().getDistance(finalWaypoints.get(i - 1).getTranslation());
      if (dist > surveyPathMetersLeft) {
        Pose2d prevPoint = finalWaypoints.get(i - 1);
        Pose2d nextPoint = finalWaypoints.get(i);
        Translation2d simRobotTranslation =
            prevPoint
                .getTranslation()
                .interpolate(nextPoint.getTranslation(), surveyPathMetersLeft / dist);
        Rotation2d simRobotRotation =
            prevPoint.getRotation().interpolate(nextPoint.getRotation(), surveyPathMetersLeft / dist);
        surveyField.getObject("RobotSim").setPose(new Pose2d(simRobotTranslation, simRobotRotation));
        break;
      }
      surveyPathMetersLeft -= dist;
      if (i == finalWaypoints.size() - 1) {
        surveyPathProgressMeters = 0.0;
      }
    }
  }
}
