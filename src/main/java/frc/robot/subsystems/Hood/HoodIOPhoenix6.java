package frc.robot.subsystems.Hood;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.Constants.HoodConstants;
import frc.robot.Constants.MotorIDs;

public class HoodIOPhoenix6 implements HoodIO {
  private final TalonFX motor = new TalonFX(MotorIDs.kHoodMotorId, CANBus.roboRIO());
  private final MotionMagicVoltage positionRequest = new MotionMagicVoltage(0.0).withEnableFOC(true);

  public HoodIOPhoenix6() {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.Feedback.SensorToMechanismRatio = HoodConstants.HoodRatio;
    config.Slot0.kP = HoodConstants.kP;
    config.Slot0.kI = HoodConstants.kI;
    config.Slot0.kD = HoodConstants.kD;
    config.Slot0.kV = HoodConstants.kV;
    config.Slot0.kS = HoodConstants.kS;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = HoodConstants.SupplyCurrentLimit;
    config.MotionMagic.MotionMagicCruiseVelocity = HoodConstants.MotionMagicMaxVelocity;
    config.MotionMagic.MotionMagicAcceleration = HoodConstants.MotionMagicAcceleration;
    config.MotorOutput.Inverted = HoodConstants.Inverted;
    motor.getConfigurator().apply(config);
    motor.setPosition(UnitsToRotations(HoodConstants.IdlePositionDegs));
  }

  private static double UnitsToRotations(double degrees) {
    return degrees / 360.0;
  }

  @Override
  public void setPosition(double positionDegs) {
    motor.setControl(positionRequest.withPosition(UnitsToRotations(positionDegs)));
  }

  @Override
  public void updateInputs(HoodIOInputs inputs) {
    inputs.motorConnected = BaseStatusSignal.refreshAll(motor.getPosition()).isOK();
    inputs.hoodPositionDegs = motor.getPosition().getValueAsDouble() * 360.0;
  }
}
