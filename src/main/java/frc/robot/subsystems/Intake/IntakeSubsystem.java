package frc.robot.subsystems.Intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;
import org.littletonrobotics.junction.Logger;

/** Rack (position) + dual intake rollers (velocity). */
public class IntakeSubsystem extends SubsystemBase {
  private static IntakeSubsystem instance;

  public static IntakeSubsystem getInstance() {
    if (instance == null) {
      instance = new IntakeSubsystem();
    }
    return instance;
  }

  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  private double targetRackRotations = IntakeConstants.RackIdleRotations;
  private double targetRollerRps = 0.0;

  private IntakeSubsystem() {
    if (Constants.currentMode == Constants.Mode.REAL) {
      io = new IntakeIOPhoenix6();
    } else {
      io = new IntakeIO() {};
    }
  }

  public void setRollerRps(double rps) {
    targetRollerRps = rps;
    io.setRollerRps(rps);
  }

  public void stopRollers() {
    setRollerRps(0.0);
  }

  public void setRackPosition(double positionRotations) {
    targetRackRotations =
        MathUtil.clamp(
            positionRotations, IntakeConstants.RackMinRotations, IntakeConstants.RackMaxRotations);
    io.setRackPosition(targetRackRotations);
  }

  public boolean isRackAtTarget() {
    return MathUtil.isNear(
        targetRackRotations,
        inputs.rackPositionRotations,
        IntakeConstants.RackPositionToleranceRotations);
  }

  public double getRackPositionRotations() {
    return inputs.rackPositionRotations;
  }

  public double getTargetRackRotations() {
    return targetRackRotations;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);
    Logger.recordOutput("Intake/TargetRollerRps", targetRollerRps);
    Logger.recordOutput("Intake/RollerVelocityRps", inputs.rollerVelocityRps);
    Logger.recordOutput("Intake/TargetRackRotations", targetRackRotations);
    Logger.recordOutput("Intake/RackPositionRotations", inputs.rackPositionRotations);
    Logger.recordOutput("Intake/RackAtTarget", isRackAtTarget());
  }
}
