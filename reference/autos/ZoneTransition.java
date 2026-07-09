package frc.robot.autos;

import static frc.robot.Constants.AutoConstants.*;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.lib.BLine.Path;
import frc.robot.lib.BLine.Path.PathElement;
import frc.robot.subsystems.Chassis.CommandSwerveDrivetrain;
import frc.robot.subsystems.Vision.VisionSubsystem;
import java.util.ArrayList;
import java.util.List;
import java.util.Set;

/** Alliance ↔ neutral zone transitions (Spartronics 4915). */
public class ZoneTransition {
  private final CommandSwerveDrivetrain drive;
  private final VisionSubsystem vision;

  public ZoneTransition(CommandSwerveDrivetrain drive, VisionSubsystem vision) {
    this.drive = drive;
    this.vision = vision;
  }

  public enum TraversalMethod {
    LEFT_TRENCH(true, false),
    RIGHT_TRENCH(true, true),
    LEFT_BUMP(false, false),
    RIGHT_BUMP(false, true);

    public final boolean isTrench;
    public final boolean isRightSide;

    TraversalMethod(boolean isTrench, boolean isRightSide) {
      this.isTrench = isTrench;
      this.isRightSide = isRightSide;
    }
  }

  public Command generateCommand(TraversalMethod method, boolean endWithSpeed) {
    return Commands.defer(
        () ->
            generateCommand(
                method, drive.getRelativePose().getX() < hubPose.getX(), endWithSpeed),
        Set.of(drive));
  }

  public Command generateCommand(
      TraversalMethod method, boolean toNeutralZone, boolean endWithSpeed) {
    return generateCommand(method, toNeutralZone, endWithSpeed, false);
  }

  public Command generateCommand(
      TraversalMethod method, boolean toNeutralZone, boolean endWithSpeed, boolean endInTrench) {
    return Commands.defer(
        () -> {
          if (method.isTrench) {
            return generateTrenchCommand(
                method.isRightSide, toNeutralZone, endWithSpeed, endInTrench);
          }
          return generateBumpCommand(method.isRightSide, toNeutralZone, endWithSpeed);
        },
        Set.of(drive));
  }

  public Command generateBumpCommand(
      boolean isRightSide, boolean toNeutralZone, boolean endWithSpeed) {
    Rotation2d lrFlip = isRightSide ? Rotation2d.kZero : Rotation2d.k180deg;
    Rotation2d ioFlip = toNeutralZone ? Rotation2d.kZero : Rotation2d.k180deg;

    Rotation2d bumpAngle =
        bumpApproachAngle.times((isRightSide == toNeutralZone) ? 1 : -1).rotateBy(ioFlip);
    if (!toNeutralZone) {
      bumpAngle = bumpAngle.plus(Rotation2d.k180deg);
    }

    List<PathElement> pathElements =
        new ArrayList<>(
            List.of(
                new Path.Waypoint(drive.getRelativePose()),
                new Path.RotationTarget(bumpAngle, 0.75),
                new Path.Waypoint(
                    new Pose2d(
                        hubPose
                            .plus(bumpTransform.rotateBy(lrFlip))
                            .plus(approachTransform.rotateBy(ioFlip)),
                        bumpAngle),
                    0.4),
                new Path.Waypoint(
                    hubPose
                        .plus(
                            Autos.surveyMode
                                ? approachTransform.rotateBy(ioFlip.plus(Rotation2d.k180deg))
                                : exitTransform.rotateBy(ioFlip))
                        .plus(bumpTransform.rotateBy(lrFlip)),
                    bumpAngle)));

    Autos.removePastPoses(drive, pathElements, toNeutralZone);

    Path path =
        new Path(pathElements, Autos.generatePathConstraintZone(bumpPathConstraints, 1, 2));

    return Commands.race(
        Commands.sequence(
            Autos.build(path, endWithSpeed ? Rotation2d.kZero.rotateBy(ioFlip) : null, drive),
            Autos.wait(0.5)),
        Commands.sequence(
            Commands.waitUntil(
                () ->
                    (drive.getRelativePose().getX() > hubPose.getX()) ^ !toNeutralZone
                        && drive.isFlatDebounced()
                        && vision.hasAnyPose()),
            Autos.wait(bumpDriveContinueTime)));
  }

  public Command generateTrenchCommand(
      boolean isRightSide, boolean toNeutralZone, boolean endWithSpeed, boolean endInTrench) {
    Rotation2d lrFlip = isRightSide ? Rotation2d.kZero : Rotation2d.k180deg;
    Rotation2d ioFlip = toNeutralZone ? Rotation2d.kZero : Rotation2d.k180deg;
    Rotation2d trenchAngle = trenchApproachAngle;

    List<PathElement> pathElements =
        new ArrayList<>(
            List.of(
                new Path.Waypoint(drive.getRelativePose()),
                new Path.RotationTarget(trenchAngle, 0.75),
                new Path.Waypoint(
                    new Pose2d(
                        hubPose
                            .plus(trenchTransform.rotateBy(lrFlip))
                            .plus(approachTransform.rotateBy(ioFlip)),
                        trenchAngle),
                    1.4),
                new Path.Waypoint(
                    new Pose2d(
                        hubPose
                            .plus(trenchTransform.rotateBy(lrFlip))
                            .plus(approachTransform.rotateBy(ioFlip).times(0.4)),
                        trenchAngle),
                    0.2),
                new Path.Waypoint(
                    hubPose
                        .plus(trenchTransform.rotateBy(lrFlip))
                        .plus(
                            endInTrench
                                ? trenchStopTransform.rotateBy(ioFlip)
                                : toNeutralZone
                                    ? trenchExitTransform.rotateBy(ioFlip.plus(Rotation2d.k180deg))
                                    : trenchAllianceExitTransform.rotateBy(
                                        ioFlip.plus(Rotation2d.k180deg))),
                    trenchAngle)));

    Autos.removePastPoses(drive, pathElements, toNeutralZone);

    Path path;
    if (toNeutralZone) {
      path =
          new Path(
              pathElements, Autos.generatePathConstraintZone(driveToCenterConstraints, 1, 2));
    } else {
      path = new Path(pathElements);
    }

    return Autos.build(path, endWithSpeed ? Rotation2d.kZero.rotateBy(ioFlip) : null, drive);
  }

  public Command generateStartingTrenchCommand(boolean isRightSide) {
    Rotation2d lrFlip = isRightSide ? Rotation2d.kZero : Rotation2d.k180deg;
    Rotation2d trenchAngle = startingTrenchApproachAngle.times(isRightSide ? 1 : -1);

    List<PathElement> pathElements =
        new ArrayList<>(
            List.of(
                new Path.Waypoint(drive.getRelativePose()),
                new Path.RotationTarget(trenchAngle, 0.75),
                new Path.Waypoint(
                    new Pose2d(
                        hubPose
                            .plus(trenchTransform.rotateBy(lrFlip))
                            .plus(approachTransform),
                        trenchAngle),
                    0.6),
                new Path.Waypoint(
                    hubPose
                        .plus(trenchTransform.rotateBy(lrFlip))
                        .plus(trenchExitTransform.rotateBy(Rotation2d.k180deg)),
                    trenchAngle)));

    Autos.removePastPoses(drive, pathElements, true);

    Path path = new Path(pathElements, driveToCenterConstraints);
    return Autos.build(path, Rotation2d.kZero, drive);
  }
}
