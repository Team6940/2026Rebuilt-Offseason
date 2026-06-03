package frc.robot.subsystems.Indexer;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.IndexerConstants;
import frc.robot.Constants.MotorIDs;
import org.littletonrobotics.junction.Logger;

/** Two feeder rollers and two indexer rollers, usually run together during a shot. */
public class IndexerSubsystem extends SubsystemBase {
  private static IndexerSubsystem instance;

  public static IndexerSubsystem getInstance() {
    if (instance == null) {
      instance = new IndexerSubsystem();
    }
    return instance;
  }

  private final TalonFX feederLeader;
  private final TalonFX feederFollower;
  private final TalonFX indexerLeader;
  private final TalonFX indexerFollower;
  private final VelocityVoltage velocityRequest = new VelocityVoltage(0.0).withEnableFOC(true);

  private boolean feeding = false;

  private IndexerSubsystem() {
    if (Constants.currentMode == Constants.Mode.REAL) {
      feederLeader = new TalonFX(MotorIDs.kIndexerFeederLeaderMotorId, CANBus.roboRIO());
      feederFollower = new TalonFX(MotorIDs.kIndexerFeederFollowerMotorId, CANBus.roboRIO());
      indexerLeader = new TalonFX(MotorIDs.kIndexerLeaderMotorId, CANBus.roboRIO());
      indexerFollower = new TalonFX(MotorIDs.kIndexerFollowerMotorId, CANBus.roboRIO());
      configureVelocityMotor(
          feederLeader, IndexerConstants.FeederRatio, IndexerConstants.FeederInverted,
          IndexerConstants.FeederkP, IndexerConstants.FeederkI, IndexerConstants.FeederkD,
          IndexerConstants.FeederkV, IndexerConstants.FeederkS,
          IndexerConstants.FeederSupplyCurrentLimit);
      configureVelocityMotor(
          indexerLeader, IndexerConstants.IndexerRatio, IndexerConstants.IndexerInverted,
          IndexerConstants.IndexerkP, IndexerConstants.IndexerkI, IndexerConstants.IndexerkD,
          IndexerConstants.IndexerkV, IndexerConstants.IndexerkS,
          IndexerConstants.IndexerSupplyCurrentLimit);
      configureFollower(feederFollower, IndexerConstants.FeederInverted);
      configureFollower(indexerFollower, IndexerConstants.IndexerInverted);
      feederFollower.setControl(
          new Follower(feederLeader.getDeviceID(), IndexerConstants.FollowerAlignment));
      indexerFollower.setControl(
          new Follower(indexerLeader.getDeviceID(), IndexerConstants.FollowerAlignment));
    } else {
      feederLeader = null;
      feederFollower = null;
      indexerLeader = null;
      indexerFollower = null;
    }
  }

  private void configureVelocityMotor(
      TalonFX motor,
      double ratio,
      com.ctre.phoenix6.signals.InvertedValue inverted,
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
    config.MotorOutput.Inverted = inverted;
    motor.getConfigurator().apply(config);
  }

  private void configureFollower(
      TalonFX motor, com.ctre.phoenix6.signals.InvertedValue inverted) {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted = inverted;
    motor.getConfigurator().apply(config);
  }

  public void feed() {
    feeding = true;
    setVelocities(IndexerConstants.FeedRps, IndexerConstants.IndexerRps);
  }

  public void stop() {
    feeding = false;
    setVelocities(0.0, 0.0);
  }

  private void setVelocities(double feederRps, double indexerRps) {
    if (feederLeader == null) {
      return;
    }
    if (feederRps == 0.0 && indexerRps == 0.0) {
      feederLeader.stopMotor();
      indexerLeader.stopMotor();
      return;
    }
    feederLeader.setControl(velocityRequest.withVelocity(feederRps));
    indexerLeader.setControl(velocityRequest.withVelocity(indexerRps));
  }

  public boolean isFeeding() {
    return feeding;
  }

  @Override
  public void periodic() {
    Logger.recordOutput("Indexer/Feeding", feeding);
  }
}
