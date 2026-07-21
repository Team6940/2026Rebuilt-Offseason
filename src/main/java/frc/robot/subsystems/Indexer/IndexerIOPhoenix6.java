package frc.robot.subsystems.Indexer;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.Constants.IndexerConstants;
import frc.robot.Constants.MotorIDs;

public class IndexerIOPhoenix6 implements IndexerIO {
  protected final TalonFX feederLeader =
      new TalonFX(MotorIDs.kIndexerFeederLeaderMotorId, CANBus.roboRIO());
  protected final TalonFX feederFollower =
      new TalonFX(MotorIDs.kIndexerFeederFollowerMotorId, CANBus.roboRIO());
  protected final TalonFX indexerLeader =
      new TalonFX(MotorIDs.kIndexerLeaderMotorId, CANBus.roboRIO());
  protected final TalonFX indexerFollower =
      new TalonFX(MotorIDs.kIndexerFollowerMotorId, CANBus.roboRIO());
  // private final VelocityVoltage velocityRequest = new VelocityVoltage(0.0).withEnableFOC(true);
  private final VelocityTorqueCurrentFOC focRequestFeeder = new VelocityTorqueCurrentFOC(0.0);
  private final VelocityTorqueCurrentFOC focRequestIndexer = new VelocityTorqueCurrentFOC(0.0);

  public IndexerIOPhoenix6() {
    configureVelocityMotor(
        feederLeader,
        IndexerConstants.FeederRatio,
        IndexerConstants.FeederInverted,
        IndexerConstants.FeederkP,
        IndexerConstants.FeederkI,
        IndexerConstants.FeederkD,
        IndexerConstants.FeederkV,
        IndexerConstants.FeederkS,
        IndexerConstants.FeederSupplyCurrentLimit,
        IndexerConstants.FeederStatorCurrentLimitEnable,
        IndexerConstants.FeederStatorCurrentLimit);
    configureVelocityMotor(
        indexerLeader,
        IndexerConstants.IndexerRatio,
        IndexerConstants.IndexerInverted,
        IndexerConstants.IndexerkP,
        IndexerConstants.IndexerkI,
        IndexerConstants.IndexerkD,
        IndexerConstants.IndexerkV,
        IndexerConstants.IndexerkS,
        IndexerConstants.IndexerSupplyCurrentLimit,
        IndexerConstants.IndexerStatorCurrentLimitEnable,
        IndexerConstants.IndexerStatorCurrentLimit);
    configureFollower(feederFollower, IndexerConstants.FeederInverted);
    configureFollower(indexerFollower, IndexerConstants.IndexerInverted);
    feederFollower.setControl(
        new Follower(feederLeader.getDeviceID(), IndexerConstants.FeederFollowerAlignment));
    indexerFollower.setControl(
        new Follower(indexerLeader.getDeviceID(), IndexerConstants.IndexerFollowerAlignment));
  }

  private void configureVelocityMotor(
      TalonFX motor,
      double ratio,
      InvertedValue inverted,
      double kP,
      double kI,
      double kD,
      double kV,
      double kS,
      double supplyLimit) {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.Feedback.SensorToMechanismRatio = ratio;
    config.Slot0.kP = kP;
    config.Slot0.kI = kI;
    config.Slot0.kD = kD;
    config.Slot0.kV = kV;
    config.Slot0.kS = kS;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = supplyLimit;
    config.CurrentLimits.StatorCurrentLimitEnable = false;
    config.MotorOutput.Inverted = inverted;
    motor.getConfigurator().apply(config);
  }

  private void configureVelocityMotor(
      TalonFX motor,
      double ratio,
      InvertedValue inverted,
      double kP,
      double kI,
      double kD,
      double kV,
      double kS,
      double supplyLimit,
      boolean enableStatorLimit,
      double statorLimit) {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.Feedback.SensorToMechanismRatio = ratio;
    config.Slot0.kP = kP;
    config.Slot0.kI = kI;
    config.Slot0.kD = kD;
    config.Slot0.kV = kV;
    config.Slot0.kS = kS;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = supplyLimit;
    config.CurrentLimits.StatorCurrentLimitEnable = enableStatorLimit;
    config.CurrentLimits.StatorCurrentLimit = statorLimit;
    config.MotorOutput.Inverted = inverted;
    motor.getConfigurator().apply(config);
  }

  private void configureFollower(TalonFX motor, InvertedValue inverted) {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted = inverted;
    motor.getConfigurator().apply(config);
  }

  @Override
  public void setFeederRps(double rps) {
    if (rps == 0.0) {
      feederLeader.stopMotor();
      return;
    }
    feederLeader.setControl(focRequestFeeder.withVelocity(rps));
  }

  @Override
  public void setIndexerRps(double rps) {
    if (rps == 0.0) {
      indexerLeader.stopMotor();
      return;
    }
    indexerLeader.setControl(focRequestIndexer.withVelocity(rps));
  }

  @Override
  public void updateInputs(IndexerIOInputs inputs) {
    inputs.feederConnected =
        BaseStatusSignal.refreshAll(
                feederLeader.getVelocity(),
                feederLeader.getSupplyCurrent(),
                feederLeader.getSupplyVoltage())
            .isOK();
    inputs.feederVelocityRps = feederLeader.getVelocity().getValueAsDouble();
    inputs.feederSupplyCurrentA = feederLeader.getSupplyCurrent().getValueAsDouble();
    inputs.feederSupplyVoltageV = feederLeader.getSupplyVoltage().getValueAsDouble();

    inputs.indexerConnected =
        BaseStatusSignal.refreshAll(
                indexerLeader.getVelocity(),
                indexerLeader.getSupplyCurrent(),
                indexerLeader.getSupplyVoltage())
            .isOK();
    inputs.indexerVelocityRps = indexerLeader.getVelocity().getValueAsDouble();
    inputs.indexerSupplyCurrentA = indexerLeader.getSupplyCurrent().getValueAsDouble();
    inputs.indexerSupplyVoltageV = indexerLeader.getSupplyVoltage().getValueAsDouble();

    inputs.feederFollowerConnected =
        BaseStatusSignal.refreshAll(
                feederFollower.getSupplyCurrent(), feederFollower.getSupplyVoltage())
            .isOK();
    inputs.feederFollowerSupplyCurrentA = feederFollower.getSupplyCurrent().getValueAsDouble();
    inputs.feederFollowerSupplyVoltageV = feederFollower.getSupplyVoltage().getValueAsDouble();

    inputs.indexerFollowerConnected =
        BaseStatusSignal.refreshAll(
                indexerFollower.getSupplyCurrent(), indexerFollower.getSupplyVoltage())
            .isOK();
    inputs.indexerFollowerSupplyCurrentA = indexerFollower.getSupplyCurrent().getValueAsDouble();
    inputs.indexerFollowerSupplyVoltageV = indexerFollower.getSupplyVoltage().getValueAsDouble();
  }
}
