package frc.robot.autos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Chassis.CommandSwerveDrivetrain;
import frc.robot.subsystems.ImprovedCommandXboxController.Button;
import frc.robot.subsystems.SuperStructure;
import frc.robot.subsystems.SuperStructure.ControlMode;
import frc.robot.subsystems.SuperStructure.IntakeMode;
import frc.robot.subsystems.Shooter.ShooterSubsystem;
import java.util.ArrayList;
import java.util.List;
import java.util.stream.Collectors;

/** Dynamic path chooser for per-step autonomous selection. */
public class AutoBuilder {
  private static final int kMaxSteps = 20;
  private static final String kChooserTopicPrefix = "Auto Chooser/Step ";
  private static final String kPreviewObjectName = "Auto Path Preview";

  private final CommandSwerveDrivetrain drive = CommandSwerveDrivetrain.getInstance();
  private final SuperStructure superStructure = SuperStructure.getInstance();
  private final ShooterSubsystem shooter = ShooterSubsystem.getInstance();
  private final SendableChooser<AutoSegment>[] stepChoosers;
  private final AutoSegment[] stepSources;

  @SuppressWarnings("unchecked")
  public AutoBuilder() {
    stepChoosers = new SendableChooser[kMaxSteps];
    stepSources = new AutoSegment[kMaxSteps];
    for (int i = 0; i < kMaxSteps; i++) {
      stepSources[i] = null;
      stepChoosers[i] = buildChooser(null);
      SmartDashboard.putData(kChooserTopicPrefix + i, stepChoosers[i]);
    }
    resolveSteps();
  }

  public Command getAutoCommand() {
    List<AutoSegment> resolved = resolveSteps();
    if (resolved.isEmpty()) {
      return Commands.waitSeconds(RobotContainer.autoDelaySeconds.get());
    }

    List<Command> commands = new ArrayList<>();
    commands.add(Commands.waitSeconds(RobotContainer.autoDelaySeconds.get()));

    AutoSegment firstSegment = resolved.get(0);
    if (firstSegment.isPathSegment()) {
      Pose2d initialPose = getStartingPose(firstSegment);
      if (initialPose != null) {
        commands.add(Commands.runOnce(() -> drive.resetPose(initialPose)));
      }
      if (RobotContainer.enableHeatup.get()) {
        commands.add(
            superStructure.runOnce(
                () -> superStructure.setShootPhase(SuperStructure.ShootPhase.HEATUP)));
        commands.add(shooter.runOnce(() -> shooter.setVelocityRps(33.3)));
      }
    }

    for (AutoSegment segment : resolved) {
      if (segment.isPathSegment()) {
        commands.add(
            Commands.sequence(
                Commands.runOnce(() -> superStructure.setIntakeMode(segment.getIntakeMode())),
                drive.followPPPath(segment.getPathName())));
      } else if (segment.isShootSegment()) {
        commands.add(getShootCommandForSegment(segment));
      }
    }

    return Commands.sequence(commands.toArray(new Command[0]));
  }

  private Command getShootCommandForSegment(AutoSegment segment) {
    if (segment == AutoSegment.SHOOT_SCORE) {
      return superStructure.getShootCommand(ControlMode.SCORE, Button.kAutoButton).withTimeout(2.0);
    }
    if (segment == AutoSegment.SHOOT_PASS) {
      return superStructure.getShootCommand(ControlMode.PASS, Button.kAutoButton).withTimeout(2.0);
    }
    return Commands.none();
  }

  private Pose2d getStartingPose(AutoSegment firstSegment) {
    if (!firstSegment.isPathSegment()) {
      return null;
    }
    var path = drive.generatePPPath(firstSegment.getPathName());
    if (path == null) {
      return null;
    }
    var poseOpt = path.getStartingHolonomicPose();
    if (poseOpt.isEmpty()) {
      return null;
    }
    Pose2d pose = poseOpt.get();
    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
      return path.flipPath().getStartingHolonomicPose().orElse(pose);
    }
    return pose;
  }

  private List<AutoSegment> resolveSteps() {
    AutoSegment lastPathSegment = null;
    List<AutoSegment> resolved = new ArrayList<>();

    for (int i = 0; i < kMaxSteps; i++) {
      AutoSegment selected = stepChoosers[i].getSelected();
      AutoSegment normalized = selected == null ? AutoSegment.UNUSED : selected;

      if (normalized == AutoSegment.UNUSED) {
        resolved.add(AutoSegment.UNUSED);
        break;
      }

      if (normalized.isShootSegment()) {
        if (lastPathSegment == null || !lastPathSegment.canShootAfter()) {
          normalized = AutoSegment.UNUSED;
        } else {
          resolved.add(normalized);
          continue;
        }
      }

      if (normalized.isPathSegment()) {
        if (lastPathSegment == null) {
          if (!AutoSegment.getStartingSegments().contains(normalized)) {
            normalized = AutoSegment.UNUSED;
          }
        } else {
          if (!lastPathSegment.getNextPaths().contains(normalized)) {
            normalized = AutoSegment.UNUSED;
          }
        }
      }

      if (normalized == AutoSegment.UNUSED) {
        resolved.add(AutoSegment.UNUSED);
        break;
      }

      resolved.add(normalized);
      lastPathSegment = normalized;
    }

    // Rebuild and publish choosers that need updating
    for (int i = 0; i < kMaxSteps; i++) {
      AutoSegment source = (i == 0) ? null : (i <= resolved.size() ? resolved.get(i - 1) : lastPathSegment);
      if (source == AutoSegment.UNUSED) source = null;

      if (stepSources[i] == source && stepChoosers[i] != null) {
        continue;
      }

      SendableChooser<AutoSegment> chooser = buildChooser(source);
      SmartDashboard.putData(kChooserTopicPrefix + i, chooser);
      stepChoosers[i] = chooser;
      stepSources[i] = source;
    }

    updatePreview(resolved);
    return resolved.stream().filter(segment -> segment != AutoSegment.UNUSED).collect(Collectors.toList());
  }

  private SendableChooser<AutoSegment> buildChooser(AutoSegment previousPathSegment) {
    SendableChooser<AutoSegment> chooser = new SendableChooser<>();
    chooser.setDefaultOption(AutoSegment.UNUSED.getDisplayName(), AutoSegment.UNUSED);
    List<AutoSegment> allowed = previousPathSegment == null ? AutoSegment.getStartingSegments() : previousPathSegment.getNextPaths();
    for (AutoSegment option : allowed) {
      chooser.addOption(option.getDisplayName(), option);
    }
    if (previousPathSegment != null && previousPathSegment.canShootAfter()) {
      chooser.addOption(AutoSegment.SHOOT_SCORE.getDisplayName(), AutoSegment.SHOOT_SCORE);
      chooser.addOption(AutoSegment.SHOOT_PASS.getDisplayName(), AutoSegment.SHOOT_PASS);
    }
    chooser.onChange(selected -> {
      if (stepChoosers.length > 0) {
        resolveSteps();
      }
    });
    return chooser;
  }

  private void updatePreview(List<AutoSegment> resolved) {
    List<Pose2d> previewPoses = new ArrayList<>();
    for (AutoSegment segment : resolved) {
      if (!segment.isPathSegment()) {
        continue;
      }
      var path = drive.generatePPPath(segment.getPathName());
      if (path == null) {
        continue;
      }
      for (int i = 0; i < path.numPoints(); i++) {
        previewPoses.add(new Pose2d(path.getPoint(i).position, new Rotation2d()));
      }
    }
    drive.getField2d().getObject(kPreviewObjectName).setPoses(previewPoses);
  }
}
