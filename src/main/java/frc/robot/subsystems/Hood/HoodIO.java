package frc.robot.subsystems.Hood;

import org.littletonrobotics.junction.AutoLog;

public interface HoodIO {
  @AutoLog
  public class HoodIOInputs {
    public boolean motorConnected = false;
    public double hoodPositionDegs = 0.0;
  }

  public default void setPosition(double positionDegs) {}

  public default void updateInputs(HoodIOInputs inputs) {}
}
