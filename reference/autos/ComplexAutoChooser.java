package frc.robot.autos;

import static frc.robot.Constants.AutoConstants.*;
import static frc.robot.autos.ComplexAutoChooser.AllowedTransitions.*;
import static frc.robot.autos.ComplexAutoChooser.AutoSegment.*;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.autos.NeutralZoneAutos.IntakeShift;
import frc.robot.autos.ZoneTransition.TraversalMethod;
import frc.robot.subsystems.ImprovedCommandXboxController.Button;
import frc.robot.subsystems.SuperStructure;
import frc.robot.subsystems.SuperStructure.ControlMode;
import frc.robot.subsystems.SuperStructure.IntakeMode;
import frc.robot.subsystems.SuperStructure.ShootPhase;
import java.util.ArrayList;
import java.util.EnumSet;
import java.util.Set;
import java.util.function.Supplier;

/** Step-based auto chooser; fixed-position {@link AutoSegment#SHOOT} only after alliance return. */
public class ComplexAutoChooser {
  private static final String HEATUP_ENABLED_KEY = "Heatup Enabled";

  public enum AutoSegment {
    L_TRENCH_TO_NEUTRAL("LT -> N", READY_TO_INTAKE),
    L_BUMP_TO_NEUTRAL("LB -> N", READY_TO_INTAKE),
    R_TRENCH_TO_NEUTRAL("RT -> N", READY_TO_INTAKE),
    R_BUMP_TO_NEUTRAL("RB -> N", READY_TO_INTAKE),

    INTAKE_QUARTER("I 1/4", DIST_SELECT),
    INTAKE_HALF("I 1/2", DIST_SELECT),
    INTAKE_HAIRPIN("I 1/4 (w/ Hairpin)", WITHIN_NEUTRAL),
    INTAKE_INVERTED_QUARTER("I Inverted 1/4", DIST_SELECT),
    INTAKE_MIDDLE("I Mid", WITHIN_NEUTRAL),
    INTAKE_SHORT("I 1/8", DIST_SELECT),
    STOP("Stop", NONE),

    INTAKE_CLOSE("Intake Close", WITHIN_NEUTRAL),
    INTAKE_NORMAL("Intake Normal", WITHIN_NEUTRAL),
    INTAKE_FAR("Intake Far", WITHIN_NEUTRAL),

    L_TRENCH_TO_ALLIANCE("LT -> A", AFTER_ALLIANCE_RETURN),
    L_BUMP_TO_ALLIANCE("LB -> A", AFTER_ALLIANCE_RETURN),
    R_TRENCH_TO_ALLIANCE("RT -> A", AFTER_ALLIANCE_RETURN),
    R_BUMP_TO_ALLIANCE("RB -> A", AFTER_ALLIANCE_RETURN),

    DEPOT("-> D", ALLIANCE_DEPLOY),
    OUTPOST("-> O", ALLIANCE_DEPLOY),
    SHOOT("Shoot", AFTER_ALLIANCE_RETURN),
    PAUSE("P", AFTER_ALLIANCE_RETURN),
    ALT_PAUSE("P-A", AFTER_ALLIANCE_RETURN),
    AT_START("", ALLIANCE_DEPLOY),
    UNUSED(" ", NONE);

    public final String userFacingName;
    private final AllowedTransitions allowedTransitions;

    AutoSegment(String userFacingName, AllowedTransitions allowedTransitions) {
      this.userFacingName = userFacingName;
      this.allowedTransitions = allowedTransitions;
    }

    public AutoSegment[] getAllowedTransitions() {
      return allowedTransitions.getAllowedSegments();
    }
  }

  public enum AllowedTransitions {
    READY_TO_INTAKE(
        () ->
            new AutoSegment[] {
              INTAKE_QUARTER,
              INTAKE_HALF,
              INTAKE_HAIRPIN,
              INTAKE_INVERTED_QUARTER,
              INTAKE_MIDDLE,
              STOP,
              INTAKE_SHORT
            }),
    DIST_SELECT(
        () -> new AutoSegment[] {INTAKE_CLOSE, INTAKE_NORMAL, INTAKE_FAR}),
    WITHIN_NEUTRAL(
        () ->
            new AutoSegment[] {
              L_TRENCH_TO_ALLIANCE,
              L_BUMP_TO_ALLIANCE,
              R_TRENCH_TO_ALLIANCE,
              R_BUMP_TO_ALLIANCE
            }),
    /** Alliance-side options before ever leaving (no fixed-position shoot). */
    ALLIANCE_DEPLOY(
        () ->
            new AutoSegment[] {
              L_TRENCH_TO_NEUTRAL,
              L_BUMP_TO_NEUTRAL,
              R_TRENCH_TO_NEUTRAL,
              R_BUMP_TO_NEUTRAL,
              DEPOT,
              OUTPOST
            }),
    /** After returning from neutral — fixed-position shoot / pause (not moving shoot). */
    AFTER_ALLIANCE_RETURN(
        () ->
            new AutoSegment[] {
              L_TRENCH_TO_NEUTRAL,
              L_BUMP_TO_NEUTRAL,
              R_TRENCH_TO_NEUTRAL,
              R_BUMP_TO_NEUTRAL,
              DEPOT,
              OUTPOST,
              SHOOT,
              PAUSE,
              ALT_PAUSE
            }),
    NONE(() -> new AutoSegment[] {});

    private final Supplier<AutoSegment[]> allowedSegmentsSupplier;
    private AutoSegment[] allowedSegments;

    AllowedTransitions(Supplier<AutoSegment[]> allowedSegmentsSupplier) {
      this.allowedSegmentsSupplier = allowedSegmentsSupplier;
    }

    public AutoSegment[] getAllowedSegments() {
      if (allowedSegments == null) {
        allowedSegments = allowedSegmentsSupplier.get();
      }
      return allowedSegments;
    }
  }

  private final ZoneTransition transitionFactory;
  private final DriveToPOI poiFactory;
  private final NeutralZoneAutos neutralZoneFactory;
  @SuppressWarnings("unused")
  private final PreAlignment preAlignmentFactory;
  private final SuperStructure superStructure;

  private double shootWaitTime;
  private double altShootWaitTime;
  private boolean heatupEnabled;

  private final AutoSegment[] selectedSegments;
  private final SendableChooser<AutoSegment>[] segmentChoosers;
  private final AutoSegment[] segmentSources;

  @SuppressWarnings("unchecked")
  public ComplexAutoChooser(
      ZoneTransition transitionFactory,
      DriveToPOI poiFactory,
      NeutralZoneAutos neutralZoneFactory,
      PreAlignment preAlignmentFactory,
      SuperStructure superStructure,
      int maxSegments) {
    this.transitionFactory = transitionFactory;
    this.poiFactory = poiFactory;
    this.neutralZoneFactory = neutralZoneFactory;
    this.preAlignmentFactory = preAlignmentFactory;
    this.superStructure = superStructure;

    selectedSegments = new AutoSegment[maxSegments];
    segmentChoosers = (SendableChooser<AutoSegment>[]) new SendableChooser<?>[maxSegments];
    segmentSources = new AutoSegment[maxSegments];
    for (int i = 0; i < maxSegments; i++) {
      selectedSegments[i] = UNUSED;
    }
    resolveSteps();

    shootWaitTime = SmartDashboard.getNumber("Auto Chooser/Pause Time", defaultShootWaitTime);
    altShootWaitTime =
        SmartDashboard.getNumber("Auto Chooser/Alt Pause Time", defaultShootWaitTime / 2);
    heatupEnabled = SmartDashboard.getBoolean("Auto Chooser/" + HEATUP_ENABLED_KEY, false);

    SmartDashboard.putNumber("Auto Chooser/Pause Time", shootWaitTime);
    SmartDashboard.putNumber("Auto Chooser/Alt Pause Time", altShootWaitTime);
    SmartDashboard.putBoolean("Auto Chooser/" + HEATUP_ENABLED_KEY, heatupEnabled);

    NetworkTable table = NetworkTableInstance.getDefault().getTable("SmartDashboard/Auto Chooser");
    table.addListener(
        "Pause Time",
        EnumSet.of(NetworkTableEvent.Kind.kValueAll),
        (eventInfo, key, event) -> shootWaitTime = event.valueData.value.getDouble());
    table.addListener(
        "Alt Pause Time",
        EnumSet.of(NetworkTableEvent.Kind.kValueAll),
        (eventInfo, key, event) -> altShootWaitTime = event.valueData.value.getDouble());
    table.addListener(
        HEATUP_ENABLED_KEY,
        EnumSet.of(NetworkTableEvent.Kind.kValueAll),
        (eventInfo, key, event) -> heatupEnabled = event.valueData.value.getBoolean());
  }

  public void resolveSteps() {
    AutoSegment lastSegment = AT_START;

    for (int i = 0; i < selectedSegments.length; i++) {
      if (segmentSources[i] == lastSegment) {
        lastSegment = selectedSegments[i] != null ? selectedSegments[i] : UNUSED;
        continue;
      }

      if (segmentChoosers[i] != null) {
        segmentChoosers[i].close();
      }

      SendableChooser<AutoSegment> segment = new SendableChooser<>();

      if (lastSegment.getAllowedTransitions().length == 0) {
        selectedSegments[i] = UNUSED;
      } else {
        for (AutoSegment option : lastSegment.getAllowedTransitions()) {
          if (option == lastSegment && option != PAUSE && option != ALT_PAUSE && option != SHOOT) {
            continue;
          }
          segment.addOption(option.userFacingName, option);
        }

        segment.setDefaultOption(UNUSED.userFacingName, UNUSED);

        final int slot = i;
        segment.onChange(
            selected -> {
              if (selectedSegments[slot] != selected) {
                selectedSegments[slot] = selected;
                resolveSteps();
              }
            });
      }

      SmartDashboard.putData("Auto Chooser/Step " + i, segment);
      segmentChoosers[i] = segment;
      segmentSources[i] = lastSegment;
      lastSegment = selectedSegments[i] != null ? selectedSegments[i] : UNUSED;
    }

    Autos.survey(Commands.defer(() -> getAuto(), Set.of()));
  }

  private static boolean isIntakeMovementSegment(AutoSegment segment) {
    return switch (segment) {
      case INTAKE_QUARTER,
          INTAKE_HALF,
          INTAKE_HAIRPIN,
          INTAKE_INVERTED_QUARTER,
          INTAKE_MIDDLE,
          INTAKE_SHORT ->
          true;
      default -> false;
    };
  }

  private static boolean isBumpSegment(AutoSegment segment) {
    return segment == L_BUMP_TO_NEUTRAL
        || segment == R_BUMP_TO_NEUTRAL
        || segment == L_BUMP_TO_ALLIANCE
        || segment == R_BUMP_TO_ALLIANCE;
  }

  private static boolean isNonMovementSegment(AutoSegment segment) {
    return segment == AT_START
        || segment == PAUSE
        || segment == ALT_PAUSE
        || segment == SHOOT
        || segment == UNUSED;
  }

  private Command withSegmentIntakeMode(Command command, AutoSegment segment) {
    if (isBumpSegment(segment)) {
      return command.beforeStarting(
          Commands.runOnce(() -> superStructure.setIntakeMode(IntakeMode.MID), superStructure));
    }
    if (isIntakeMovementSegment(segment)) {
      return command.beforeStarting(
          Commands.runOnce(() -> superStructure.setIntakeMode(IntakeMode.INTAKE), superStructure));
    }
    return command;
  }

  private boolean isRight(AutoSegment segment) {
    return segment.userFacingName.charAt(0) == 'R';
  }

  private int getNextNonPauseIndex(int currentIndex) {
    for (int i = currentIndex + 1; i < selectedSegments.length; i++) {
      if (!isNonMovementSegment(selectedSegments[i])) {
        return i;
      }
    }
    return -1;
  }

  private AutoSegment getNextNonPauseSegment(int currentIndex) {
    int index = getNextNonPauseIndex(currentIndex);
    return index != -1
        ? selectedSegments[index] != null ? selectedSegments[index] : UNUSED
        : UNUSED;
  }

  public Command getAuto() {
    ArrayList<Command> commands = new ArrayList<>();
    AutoSegment prevSegment = UNUSED;
    boolean fullHeatup = SmartDashboard.getBoolean("Auto Chooser/" + HEATUP_ENABLED_KEY, false);

    if (fullHeatup) {
      commands.add(
          Commands.runOnce(() -> superStructure.setShootPhase(ShootPhase.HEATUP), superStructure));
    }

    for (int i = 0; i < selectedSegments.length; i++) {
      AutoSegment currentSegment = selectedSegments[i];
      AutoSegment futureSegment =
          i + 1 < selectedSegments.length ? selectedSegments[i + 1] : UNUSED;
      IntakeShift intakeShift = NeutralZoneAutos.convertToIntakeShift(futureSegment);

      if (currentSegment == null || currentSegment == UNUSED || currentSegment == AT_START) {
        break;
      }

      boolean useStartingCommand =
          (i == getNextNonPauseIndex(-1))
              && (currentSegment == L_TRENCH_TO_NEUTRAL || currentSegment == R_TRENCH_TO_NEUTRAL);
      TraversalMethod nextMethod =
          PreAlignment.convertToTraversalMethod(
              i + 2 < selectedSegments.length ? selectedSegments[i + 2] : null);

      switch (currentSegment) {
        case L_TRENCH_TO_NEUTRAL ->
            commands.add(
                withSegmentIntakeMode(
                    useStartingCommand
                        ? transitionFactory.generateStartingTrenchCommand(false)
                        : transitionFactory.generateCommand(
                            TraversalMethod.LEFT_TRENCH, true, true),
                    currentSegment));
        case L_BUMP_TO_NEUTRAL ->
            commands.add(
                withSegmentIntakeMode(
                    transitionFactory.generateCommand(TraversalMethod.LEFT_BUMP, true, false),
                    currentSegment));
        case R_TRENCH_TO_NEUTRAL ->
            commands.add(
                withSegmentIntakeMode(
                    useStartingCommand
                        ? transitionFactory.generateStartingTrenchCommand(true)
                        : transitionFactory.generateCommand(
                            TraversalMethod.RIGHT_TRENCH, true, true),
                    currentSegment));
        case R_BUMP_TO_NEUTRAL ->
            commands.add(
                withSegmentIntakeMode(
                    transitionFactory.generateCommand(TraversalMethod.RIGHT_BUMP, true, false),
                    currentSegment));
        case INTAKE_CLOSE, INTAKE_NORMAL, INTAKE_FAR -> {}
        case INTAKE_QUARTER ->
            commands.add(
                withSegmentIntakeMode(
                    neutralZoneFactory.generateQuadrantCommand(
                        isRight(prevSegment),
                        PreAlignment.convertToTraversalMethod(prevSegment).isTrench,
                        intakeShift),
                    currentSegment));
        case INTAKE_HALF ->
            commands.add(
                withSegmentIntakeMode(
                    neutralZoneFactory.generateHalfCommand(
                        isRight(prevSegment),
                        PreAlignment.convertToTraversalMethod(prevSegment).isTrench,
                        nextMethod == null || nextMethod.isTrench,
                        intakeShift),
                    currentSegment));
        case INTAKE_INVERTED_QUARTER ->
            commands.add(
                withSegmentIntakeMode(
                    neutralZoneFactory.generateInvertedQuadrantCommand(
                        !isRight(prevSegment), nextMethod == null || nextMethod.isTrench, intakeShift),
                    currentSegment));
        case INTAKE_HAIRPIN ->
            commands.add(
                withSegmentIntakeMode(
                    neutralZoneFactory.generateHairpinCommand(
                        isRight(prevSegment),
                        PreAlignment.convertToTraversalMethod(prevSegment).isTrench),
                    currentSegment));
        case INTAKE_MIDDLE ->
            commands.add(
                withSegmentIntakeMode(neutralZoneFactory.generateMiddleCommand(), currentSegment));
        case INTAKE_SHORT ->
            commands.add(
                withSegmentIntakeMode(
                    neutralZoneFactory.generateShortCommand(
                        isRight(prevSegment),
                        PreAlignment.convertToTraversalMethod(prevSegment).isTrench,
                        intakeShift),
                    currentSegment));
        case STOP ->
            commands.add(neutralZoneFactory.generateStopCommand(isRight(prevSegment)));
        case L_TRENCH_TO_ALLIANCE,
            L_BUMP_TO_ALLIANCE,
            R_TRENCH_TO_ALLIANCE,
            R_BUMP_TO_ALLIANCE -> {
          boolean endWithSpeed =
              !isNonMovementSegment(futureSegment) && futureSegment != STOP;
          TraversalMethod currentTraversalMethod =
              PreAlignment.convertToTraversalMethod(currentSegment);
          boolean endInTrench =
              currentTraversalMethod
                      == PreAlignment.convertToTraversalMethod(getNextNonPauseSegment(i))
                  || getNextNonPauseSegment(i) == UNUSED;

          commands.add(
              withSegmentIntakeMode(
                  transitionFactory.generateCommand(
                      currentTraversalMethod, false, endWithSpeed, endInTrench),
                  currentSegment));
        }
        case DEPOT -> commands.add(poiFactory.generateCommand(DriveToPOI.POI.DEPOT));
        case OUTPOST -> commands.add(poiFactory.generateCommand(DriveToPOI.POI.OUTPOST));
        case SHOOT -> {
          if (!fullHeatup) {
            commands.add(
                Commands.runOnce(
                    () -> superStructure.setShootPhase(ShootPhase.HEATUP), superStructure));
          }
          commands.add(
              superStructure
                  .getShootCommand(ControlMode.SCORE, Button.kAutoButton)
                  .withTimeout(shootWaitTime));
          commands.add(
              Commands.runOnce(() -> superStructure.setIntakeMode(IntakeMode.MID), superStructure));
          if (fullHeatup) {
            commands.add(
                Commands.runOnce(
                    () -> superStructure.setShootPhase(ShootPhase.HEATUP), superStructure));
          }
        }
        case ALT_PAUSE ->
            commands.add(
                superStructure
                    .getShootCommand(ControlMode.SCORE, Button.kAutoButton)
                    .withTimeout(altShootWaitTime));
        case PAUSE ->
            commands.add(
                Commands.deadline(
                    superStructure
                        .getShootCommand(ControlMode.SCORE, Button.kAutoButton)
                        .withTimeout(shootWaitTime)));
        case UNUSED -> System.out.println("Chat what are we doing?");
      }

      prevSegment = currentSegment;
    }

    if (fullHeatup) {
      commands.add(
          Commands.runOnce(() -> superStructure.setShootPhase(ShootPhase.OFF), superStructure));
    }

    return new SequentialCommandGroup(commands.toArray(Command[]::new));
  }
}
