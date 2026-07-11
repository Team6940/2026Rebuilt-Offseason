package frc.robot.subsystems.Shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
  @AutoLog
  public class ShooterIOInputs {
    public boolean leaderAConnected = false;
    public double leaderAVelocityRps = 0.0;
    public double leaderASupplyVoltageV = 0.0;
    public double leaderASupplyCurrentA = 0.0;
    public boolean leaderBConnected = false;
    public double leaderBVelocityRps = 0.0;
    public double leaderBSupplyVoltageV = 0.0;
    public double leaderBSupplyCurrentA = 0.0;
    public boolean followerAConnected = false;
    public double followerASupplyVoltageV = 0.0;
    public double followerASupplyCurrentA = 0.0;
    public boolean followerBConnected = false;
    public double followerBSupplyVoltageV = 0.0;
    public double followerBSupplyCurrentA = 0.0;
  }

  public default void setRps(double rps) {}

  /** Sim-only: launch one FUEL from the simulated intake using current hood and shooter speed. */
  public default boolean simulateLaunch(double hoodDegs, double shooterRps) {
    return false;
  }

  public default void updateInputs(ShooterIOInputs inputs) {}
}
