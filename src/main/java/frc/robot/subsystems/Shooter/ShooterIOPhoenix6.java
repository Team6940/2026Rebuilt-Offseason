package frc.robot.subsystems.Shooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.Constants.MotorIDs;
import frc.robot.Constants.ShooterConstants;

public class ShooterIOPhoenix6 implements ShooterIO {
  protected final TalonFX leaderA =
      new TalonFX(MotorIDs.kShooterLeaderMotorIdA, CANBus.roboRIO());
  protected final TalonFX followerA =
      new TalonFX(MotorIDs.kShooterFollowerMotorIdA, CANBus.roboRIO());
  protected final TalonFX leaderB =
      new TalonFX(MotorIDs.kShooterLeaderMotorIdB, CANBus.roboRIO());
  protected final TalonFX followerB =
      new TalonFX(MotorIDs.kShooterFollowerMotorIdB, CANBus.roboRIO());
  private final VelocityVoltage velocityRequest = new VelocityVoltage(0.0).withEnableFOC(true);

  public ShooterIOPhoenix6() {
    configureLeader(leaderA);
    configureLeader(leaderB);
    configureFollower(followerA);
    configureFollower(followerB);
    followerA.setControl(
        new Follower(leaderA.getDeviceID(), ShooterConstants.FollowerAlignment));
    followerB.setControl(
        new Follower(leaderB.getDeviceID(), ShooterConstants.FollowerAlignment));
  }

  private void configureLeader(TalonFX motor) {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.Feedback.SensorToMechanismRatio = ShooterConstants.ShooterRatio;
    config.Slot0.kP = ShooterConstants.kP;
    config.Slot0.kI = ShooterConstants.kI;
    config.Slot0.kD = ShooterConstants.kD;
    config.Slot0.kV = ShooterConstants.kV;
    config.Slot0.kS = ShooterConstants.kS;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = ShooterConstants.SupplyCurrentLimit;
    config.CurrentLimits.StatorCurrentLimit = ShooterConstants.StatorCurrentLimit;
    config.MotorOutput.Inverted = ShooterConstants.Inverted;
    motor.getConfigurator().apply(config);
  }

  private void configureFollower(TalonFX motor) {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.MotorOutput.Inverted = ShooterConstants.Inverted;
    motor.getConfigurator().apply(config);
  }

  @Override
  public void setRps(double rps) {
    if (rps == 0.0) {
      leaderA.stopMotor();
      leaderB.stopMotor();
      return;
    }
    leaderA.setControl(velocityRequest.withVelocity(rps));
    leaderB.setControl(velocityRequest.withVelocity(rps));
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    inputs.leaderAConnected =
        BaseStatusSignal.refreshAll(leaderA.getVelocity()).isOK();
    inputs.leaderAVelocityRps = leaderA.getVelocity().getValueAsDouble();

    inputs.leaderBConnected =
        BaseStatusSignal.refreshAll(leaderB.getVelocity()).isOK();
    inputs.leaderBVelocityRps = leaderB.getVelocity().getValueAsDouble();
  }
}
