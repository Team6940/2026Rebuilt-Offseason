package frc.robot.subsystems.Intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.MotorIDs;

public class IntakeIOPhoenix6 implements IntakeIO {
  protected final TalonFX rackMotor = new TalonFX(MotorIDs.kIntakeRackMotorId, CANBus.roboRIO());
  protected final TalonFX rollerLeader =
      new TalonFX(MotorIDs.kIntakeLeaderMotorId, CANBus.roboRIO());
  protected final TalonFX rollerFollower =
      new TalonFX(MotorIDs.kIntakeFollowerMotorId, CANBus.roboRIO());

  private final MotionMagicVoltage rackPositionRequest =
      new MotionMagicVoltage(0.).withEnableFOC(true);
  private final VelocityVoltage rollerVelocityRequest = new VelocityVoltage(0).withEnableFOC(true);

  public IntakeIOPhoenix6() {
    configureRack();
    configureRollers();
    rollerFollower.setControl(
        new Follower(rollerLeader.getDeviceID(), IntakeConstants.RollerFollowerAlignment));
  }

  private void configureRack() {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.Feedback.SensorToMechanismRatio = IntakeConstants.RackRatio;
    config.Voltage.PeakForwardVoltage = 12.0;
    config.Voltage.PeakReverseVoltage = -12.0;
    config.Slot0.kP = IntakeConstants.RackkP;
    config.Slot0.kI = IntakeConstants.RackkI;
    config.Slot0.kD = IntakeConstants.RackkD;
    config.Slot0.kV = IntakeConstants.RackkV;
    config.Slot0.kS = IntakeConstants.RackkS;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = IntakeConstants.RackSupplyCurrentLimit;
    config.CurrentLimits.StatorCurrentLimit = IntakeConstants.RackStatorCurrentLimit;
    config.MotorOutput.Inverted = IntakeConstants.RackInverted;
    config.MotionMagic.MotionMagicCruiseVelocity = IntakeConstants.RackMotionMagicMaxVelocity;
    config.MotionMagic.MotionMagicAcceleration = IntakeConstants.RackMotionMagicAcceleration;
    rackMotor.getConfigurator().apply(config);
    rackMotor.setPosition(IntakeConstants.RackIdleRotations);
  }

  private void configureRollers() {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.Feedback.SensorToMechanismRatio = IntakeConstants.RollerRatio;
    config.Voltage.PeakForwardVoltage = 12.0;
    config.Voltage.PeakReverseVoltage = -12.0;
    config.Slot0.kP = IntakeConstants.RollerkP;
    config.Slot0.kI = IntakeConstants.RollerkI;
    config.Slot0.kD = IntakeConstants.RollerkD;
    config.Slot0.kV = IntakeConstants.RollerkV;
    config.Slot0.kS = IntakeConstants.RollerkS;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = IntakeConstants.RollerSupplyCurrentLimit;
    config.CurrentLimits.StatorCurrentLimitEnable = false;
    config.MotorOutput.Inverted = IntakeConstants.RollerInverted;
    rollerLeader.getConfigurator().apply(config);
    TalonFXConfiguration followerConfig = new TalonFXConfiguration();
    followerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    followerConfig.MotorOutput.Inverted = IntakeConstants.RollerInverted;
    rollerFollower.getConfigurator().apply(followerConfig);
  }

  @Override
  public void setRollerRps(double rps) {
    if (rps == 0.0) {
      rollerLeader.stopMotor();
      return;
    }
    rollerLeader.setControl(rollerVelocityRequest.withVelocity(rps));
  }

  @Override
  public void setRackPosition(double positionRotations) {
    rackMotor.setControl(rackPositionRequest.withPosition(positionRotations));
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.rackConnected =
        BaseStatusSignal.refreshAll(
                rackMotor.getPosition(), rackMotor.getSupplyCurrent(), rackMotor.getSupplyVoltage())
            .isOK();
    inputs.rackPositionRotations = rackMotor.getPosition().getValueAsDouble();
    inputs.rackSupplyCurrentA = rackMotor.getSupplyCurrent().getValueAsDouble();
    inputs.rackSupplyVoltageV = rackMotor.getSupplyVoltage().getValueAsDouble();

    inputs.rollerConnected =
        BaseStatusSignal.refreshAll(
                rollerLeader.getVelocity(),
                rollerLeader.getSupplyCurrent(),
                rollerLeader.getSupplyVoltage())
            .isOK();
    inputs.rollerVelocityRps = rollerLeader.getVelocity().getValueAsDouble();
    inputs.rollerSupplyCurrentA = rollerLeader.getSupplyCurrent().getValueAsDouble();
    inputs.rollerSupplyVoltageV = rollerLeader.getSupplyVoltage().getValueAsDouble();

    inputs.rollerFollowerConnected =
        BaseStatusSignal.refreshAll(
                rollerFollower.getSupplyCurrent(), rollerFollower.getSupplyVoltage())
            .isOK();
    inputs.rollerFollowerSupplyCurrentA = rollerFollower.getSupplyCurrent().getValueAsDouble();
    inputs.rollerFollowerSupplyVoltageV = rollerFollower.getSupplyVoltage().getValueAsDouble();
  }
}
