package frc.robot.subsystems.Shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
  @AutoLog
  public class ShooterIOInputs {
    public boolean leaderAConnected = false;
    public double leaderAVelocityRps = 0.0;
    public boolean leaderBConnected = false;
    public double leaderBVelocityRps = 0.0;
  }

  public default void setRps(double rps) {}

  public default void updateInputs(ShooterIOInputs inputs) {}
}
