package frc.robot.subsystems.Vision;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.subsystems.Chassis.CommandSwerveDrivetrain;
import org.littletonrobotics.junction.AutoLog;

/** Logged per-camera vision inputs for AdvantageKit replay. */
public interface VisionIO {
  @AutoLog
  class VisionCameraInputs {
    public boolean connected = false;
    public boolean seesTarget = false;
    public boolean hasMeasurement = false;
    public Pose2d pose = new Pose2d();
    public double timestampSeconds = 0.0;
    public double stdDevX = 0.0;
    public double stdDevY = 0.0;
    public double stdDevYawRad = 0.0;
    public int tagCount = 0;
    public int tagIdCount = 0;
    public int tagId0 = -1;
    public int tagId1 = -1;
    public int tagId2 = -1;
    public int tagId3 = -1;

    /** {@link VisionSubsystem.MeasurementSource} ordinal. */
    public int measurementSource = 0;

    public String sourceLabel = "";
    public String stdDevSource = "";
  }

  default void updateInputs(
      VisionCameraInputs limelightLeft,
      VisionCameraInputs limelightRight,
      VisionCameraInputs photonFront,
      CommandSwerveDrivetrain drive,
      double fpgaNow) {}
}
