package frc.robot.subsystems.Chassis;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import org.littletonrobotics.junction.AutoLog;

/** Logged swerve odometry inputs for AdvantageKit replay (CTRE SwerveDrivetrain wrapper). */
public interface SwerveDriveIO {
  @AutoLog
  class SwerveDriveIOInputs {
    public Pose2d pose = new Pose2d();
    public ChassisSpeeds chassisSpeeds = new ChassisSpeeds();
    public double pigeonYawDeg = 0.0;
    public double pigeonPitchDeg = 0.0;
    public double pigeonRollDeg = 0.0;
    public double pigeonYawRateDegPerSec = 0.0;
  }

  default void updateInputs(SwerveDriveIOInputs inputs) {}
}
