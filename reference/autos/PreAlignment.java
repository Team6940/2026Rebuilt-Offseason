package frc.robot.autos;

import static frc.robot.Constants.AutoConstants.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.autos.ComplexAutoChooser.AutoSegment;
import frc.robot.autos.ZoneTransition.TraversalMethod;
import frc.robot.subsystems.Chassis.CommandSwerveDrivetrain;
import java.util.Map;
import java.util.Set;

/** Pre-alignment between zone transitions (Spartronics 4915). */
public class PreAlignment {
  private final CommandSwerveDrivetrain drive;

  public PreAlignment(CommandSwerveDrivetrain drive) {
    this.drive = drive;
  }

  private static final Map<AutoSegment, TraversalMethod> segmentToTraversalMethodMap =
      Map.of(
          AutoSegment.L_TRENCH_TO_NEUTRAL, TraversalMethod.LEFT_TRENCH,
          AutoSegment.L_BUMP_TO_NEUTRAL, TraversalMethod.LEFT_BUMP,
          AutoSegment.R_TRENCH_TO_NEUTRAL, TraversalMethod.RIGHT_TRENCH,
          AutoSegment.R_BUMP_TO_NEUTRAL, TraversalMethod.RIGHT_BUMP,
          AutoSegment.L_TRENCH_TO_ALLIANCE, TraversalMethod.LEFT_TRENCH,
          AutoSegment.L_BUMP_TO_ALLIANCE, TraversalMethod.LEFT_BUMP,
          AutoSegment.R_TRENCH_TO_ALLIANCE, TraversalMethod.RIGHT_TRENCH,
          AutoSegment.R_BUMP_TO_ALLIANCE, TraversalMethod.RIGHT_BUMP);

  public static TraversalMethod convertToTraversalMethod(AutoSegment segment) {
    if (segment == null) {
      return null;
    }
    return segmentToTraversalMethodMap.get(segment);
  }

  public Command generateCommand(AutoSegment prevSegment, AutoSegment nextSegment) {
    return Commands.defer(
        () -> {
          TraversalMethod prevMethod = convertToTraversalMethod(prevSegment);
          TraversalMethod nextMethod = convertToTraversalMethod(nextSegment);

          if (prevMethod == null || nextMethod == null) {
            return Commands.none();
          }

          if (prevMethod.isRightSide == nextMethod.isRightSide
              && prevMethod.isTrench == nextMethod.isTrench) {
            return Commands.none();
          }

          double lrFlip = nextMethod.isRightSide ? 0.0 : -180.0;

          Translation2d trans =
              hubPose
                  .plus(
                      (nextMethod.isTrench ? trenchTransform : bumpTransform)
                          .rotateBy(Rotation2d.fromDegrees(lrFlip)))
                  .plus(approachTransform);

          Rotation2d rotation;
          if (nextMethod.isTrench) {
            rotation = trenchApproachAngle.rotateBy(Rotation2d.fromDegrees(lrFlip));
          } else {
            rotation = bumpApproachAngle.times(nextMethod.isRightSide ? 1 : -1);
          }

          return Autos.generatePathFromWaypoint(drive, trans, rotation, alignPathConstraints);
        },
        Set.of(drive));
  }
}
