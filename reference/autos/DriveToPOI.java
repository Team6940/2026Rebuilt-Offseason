package frc.robot.autos;

import static frc.robot.Constants.AutoConstants.*;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Chassis.CommandSwerveDrivetrain;
import java.util.EnumSet;
import java.util.Set;

/** Drive to depot / outpost POIs (Spartronics 4915). */
public class DriveToPOI {
  private final CommandSwerveDrivetrain drive;
  private double outpostWaitTime;

  public DriveToPOI(CommandSwerveDrivetrain drive) {
    this.drive = drive;

    outpostWaitTime =
        SmartDashboard.getNumber("Auto Chooser/Outpost Wait Time", defaultOutpostWaitTime);
    SmartDashboard.putNumber("Auto Chooser/Outpost Wait Time", outpostWaitTime);

    NetworkTable table = NetworkTableInstance.getDefault().getTable("SmartDashboard/Auto Chooser");
    table.addListener(
        "Outpost Wait Time",
        EnumSet.of(NetworkTableEvent.Kind.kValueAll),
        (eventInfo, key, event) -> outpostWaitTime = event.valueData.value.getDouble());
  }

  public enum POI {
    DEPOT,
    OUTPOST
  }

  public Command generateCommand(POI poi) {
    return Commands.defer(
        () -> {
          if (drive.getRelativePose().getX() > hubPose.getX()) {
            return Commands.runOnce(
                () -> System.out.println("Past hub, cannot drive to POI"));
          }

          return switch (poi) {
            case DEPOT ->
                Autos.generatePathFromWaypoint(
                    drive,
                    depotPose.plus(
                        new Translation2d(robotLength.in(Meters) / 2.0 + intakeLength.in(Meters), 0)),
                    Rotation2d.fromDegrees(180.0));
            case OUTPOST ->
                Commands.sequence(
                    Autos.generatePathFromWaypoint(
                        drive,
                        outpostPose
                            .plus(new Translation2d(robotWidth.in(Meters) / 2.0, 0))
                            .plus(new Translation2d(outpostPadding.in(Meters), 0)),
                        Rotation2d.fromDegrees(90.0)),
                    Commands.runOnce(drive::lockModules, drive),
                    Autos.wait(outpostWaitTime));
          };
        },
        Set.of(drive));
  }
}
