package frc.robot.subsystems.Intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.MotorIDs;
import org.littletonrobotics.junction.Logger;

/** Rack (position) + dual intake rollers (velocity), no IO layer. */
public class IntakeSubsystem extends SubsystemBase {
  private static IntakeSubsystem instance;

  public static IntakeSubsystem getInstance() {
    if (instance == null) {
      instance = new IntakeSubsystem();
    }
    return instance;
  }

  private final TalonFX rackMotor;
  private final TalonFX rollerLeader;
  private final TalonFX rollerFollower;

  private final MotionMagicVoltage rackPositionRequest =
      new MotionMagicVoltage(0.).withEnableFOC(true);
  private final VelocityVoltage rollerVelocityRequest =
      new VelocityVoltage(0).withEnableFOC(true);

  private double targetRackRotations = IntakeConstants.RackIdleRotations;
  private double targetRollerRps = 0.0;
  private double rackPositionRotations = 0.0;
  private double rollerVelocityRps = 0.0;

  private IntakeSubsystem() {
    if (Constants.currentMode == Constants.Mode.REAL) {
      rackMotor = new TalonFX(MotorIDs.kIntakeRackMotorId, CANBus.roboRIO());
      rollerLeader = new TalonFX(MotorIDs.kIntakeLeaderMotorId, CANBus.roboRIO());
      rollerFollower = new TalonFX(MotorIDs.kIntakeFollowerMotorId, CANBus.roboRIO());
      configureRack();
      configureRollers();
      rollerFollower.setControl(
          new Follower(rollerLeader.getDeviceID(), IntakeConstants.RollerFollowerAlignment));
    } else {
      rackMotor = null;
      rollerLeader = null;
      rollerFollower = null;
    }
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
    config.MotorOutput.Inverted = IntakeConstants.RollerInverted;
    rollerLeader.getConfigurator().apply(config);
    TalonFXConfiguration followerConfig = new TalonFXConfiguration();
    followerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    followerConfig.MotorOutput.Inverted = IntakeConstants.RollerInverted;
    rollerFollower.getConfigurator().apply(followerConfig);
  }

  public void setRollerRps(double rps) {
    targetRollerRps = rps;
    if (rollerLeader == null) {
      return;
    }
    if (rps == 0.0) {
      rollerLeader.stopMotor();
    } else {
      rollerLeader.setControl(rollerVelocityRequest.withVelocity(rps));
    }
  }

  public void stopRollers() {
    setRollerRps(0.0);
  }

  public void setRackPosition(double positionRotations) {
    targetRackRotations =
        MathUtil.clamp(
            positionRotations, IntakeConstants.RackMinRotations, IntakeConstants.RackMaxRotations);
    if (rackMotor == null) {
      rackPositionRotations = targetRackRotations;
      return;
    }
    rackMotor.setControl(rackPositionRequest.withPosition(targetRackRotations));
  }

  public boolean isRackAtTarget() {
    return MathUtil.isNear(
        targetRackRotations,
        rackPositionRotations,
        IntakeConstants.RackPositionToleranceRotations);
  }

  public double getRackPositionRotations() {
    return rackPositionRotations;
  }

  public double getTargetRackRotations() {
    return targetRackRotations;
  }

  @Override
  public void periodic() {
    if (rackMotor != null) {
      BaseStatusSignal.refreshAll(rackMotor.getPosition());
      rackPositionRotations = rackMotor.getPosition().getValueAsDouble();
    }
    if (rollerLeader != null) {
      BaseStatusSignal.refreshAll(rollerLeader.getVelocity());
      rollerVelocityRps = rollerLeader.getVelocity().getValueAsDouble();
    }

    Logger.recordOutput("Intake/TargetRollerRps", targetRollerRps);
    Logger.recordOutput("Intake/RollerVelocityRps", rollerVelocityRps);
    Logger.recordOutput("Intake/TargetRackRotations", targetRackRotations);
    Logger.recordOutput("Intake/RackPositionRotations", rackPositionRotations);
    Logger.recordOutput("Intake/RackAtTarget", isRackAtTarget());
  }
}
