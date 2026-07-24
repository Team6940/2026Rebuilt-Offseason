package frc.robot.subsystems.Intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  @AutoLog
  public class IntakeIOInputs {
    public boolean rackConnected = false;
    public double rackPositionRotations = 0.0;
    public double rackSupplyVoltageV = 0.0;
    public double rackSupplyCurrentA = 0.0;
    public boolean rollerConnected = false;
    public double rollerVelocityRps = 0.0;
    public double rollerSupplyVoltageV = 0.0;
    public double rollerSupplyCurrentA = 0.0;
    public boolean rollerFollowerConnected = false;
    public double rollerFollowerSupplyVoltageV = 0.0;
    public double rollerFollowerSupplyCurrentA = 0.0;
    public int fuelInIntakeCount = 0;
  }

  public default void setRollerRps(double rps) {}

  public default void setRackPosition(double positionRotations) {}

  public default void updateInputs(IntakeIOInputs inputs) {}
}
