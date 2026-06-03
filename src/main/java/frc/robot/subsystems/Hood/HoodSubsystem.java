package frc.robot.subsystems.Hood;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.HoodConstants;
import frc.robot.Constants.MotorIDs;
import org.littletonrobotics.junction.Logger;

/** Hood angle in degrees with auto setpoint plus operator scalar trim. */
public class HoodSubsystem extends SubsystemBase {
  private static HoodSubsystem instance;

  public static HoodSubsystem getInstance() {
    if (instance == null) {
      instance = new HoodSubsystem();
    }
    return instance;
  }

  private final TalonFX motor;
  private final MotionMagicVoltage positionRequest = new MotionMagicVoltage(0.0).withEnableFOC(true);

  private double autoSetpointDegs = HoodConstants.IdlePositionDegs;
  private double operatorInputScalar = 0.0;
  private double targetPositionDegs = HoodConstants.IdlePositionDegs;
  private double positionDegs = HoodConstants.IdlePositionDegs;

  private HoodSubsystem() {
    if (Constants.currentMode == Constants.Mode.REAL) {
      motor = new TalonFX(MotorIDs.kHoodMotorId, CANBus.roboRIO());
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
    } else {
      motor = null;
    }
  }

  private static double UnitsToRotations(double degrees) {
    return degrees / 360.0;
  }

  public void setAutoSetpoint(double positionDegrees) {
    autoSetpointDegs = clamp(positionDegrees);
  }

  public void setOperatorInputScalar(double scalar) {
    operatorInputScalar = MathUtil.clamp(scalar, -1.0, 1.0);
  }

  public void setIdle() {
    autoSetpointDegs = HoodConstants.IdlePositionDegs;
    operatorInputScalar = 0.0;
    applyTarget();
  }

  public double getTargetPositionDegs() {
    return targetPositionDegs;
  }

  public double getPositionDegs() {
    return positionDegs;
  }

  private void applyTarget() {
    targetPositionDegs =
        clamp(autoSetpointDegs + operatorInputScalar * HoodConstants.HybridRangeDegs);
    if (motor == null) {
      positionDegs = targetPositionDegs;
      return;
    }
    motor.setControl(positionRequest.withPosition(UnitsToRotations(targetPositionDegs)));
  }

  private double clamp(double positionDegrees) {
    return MathUtil.clamp(positionDegrees, HoodConstants.MinDegs, HoodConstants.MaxDegs);
  }

  @Override
  public void periodic() {
    applyTarget();
    if (motor != null) {
      BaseStatusSignal.refreshAll(motor.getPosition());
      positionDegs = motor.getPosition().getValueAsDouble() * 360.0;
    }

    Logger.recordOutput("Hood/TargetPositionDegs", targetPositionDegs);
    Logger.recordOutput("Hood/PositionDegs", positionDegs);
    Logger.recordOutput("Hood/AutoSetpointDegs", autoSetpointDegs);
    Logger.recordOutput("Hood/OperatorInputScalar", operatorInputScalar);
  }
}
