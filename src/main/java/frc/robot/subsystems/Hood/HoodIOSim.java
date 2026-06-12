package frc.robot.subsystems.Hood;

import frc.robot.Constants.HoodConstants;

/** Ideal hood sim: tracks setpoint without maple-sim motor physics oscillation. */
public class HoodIOSim implements HoodIO {
  private double positionDegs = HoodConstants.IdlePositionDegs;

  @Override
  public void setPosition(double positionDegs) {
    this.positionDegs = positionDegs;
  }

  @Override
  public void updateInputs(HoodIOInputs inputs) {
    inputs.motorConnected = true;
    inputs.hoodPositionDegs = positionDegs;
  }
}
