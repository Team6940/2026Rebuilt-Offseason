package frc.robot.subsystems.Hood;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.HoodConstants;
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

  private final HoodIO io;
  private final HoodIOInputsAutoLogged inputs = new HoodIOInputsAutoLogged();

  private double autoSetpointDegs = HoodConstants.IdlePositionDegs;
  private double operatorInputScalar = 0.0;
  private double targetPositionDegs = HoodConstants.IdlePositionDegs;

  private HoodSubsystem() {
    if (Constants.currentMode == Constants.Mode.REAL) {
      io = new HoodIOPhoenix6();
    } else {
      io = new HoodIO() {};
    }
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
  }

  public double getTargetPositionDegs() {
    return targetPositionDegs;
  }

  public double getPositionDegs() {
    return inputs.hoodPositionDegs;
  }

  private void applyTarget() {
    targetPositionDegs =
        clamp(autoSetpointDegs + operatorInputScalar * HoodConstants.HybridRangeDegs);
    io.setPosition(targetPositionDegs);
  }

  private double clamp(double positionDegrees) {
    return MathUtil.clamp(positionDegrees, HoodConstants.MinDegs, HoodConstants.MaxDegs);
  }

  @Override
  public void periodic() {
    applyTarget();
    io.updateInputs(inputs);
    Logger.processInputs("Hood", inputs);
    Logger.recordOutput("Hood/TargetPositionDegs", targetPositionDegs);
    Logger.recordOutput("Hood/PositionDegs", inputs.hoodPositionDegs);
    Logger.recordOutput("Hood/AutoSetpointDegs", autoSetpointDegs);
    Logger.recordOutput("Hood/OperatorInputScalar", operatorInputScalar);
  }
}
