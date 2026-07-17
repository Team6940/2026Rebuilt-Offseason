package frc.robot.subsystems.Indexer;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.IndexerConstants;
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

  private final IndexerIO io;
  private final IndexerIOInputsAutoLogged inputs = new IndexerIOInputsAutoLogged();

  private boolean feeding = false;
  private double requestedFeederRps = 0.0;
  private double requestedIndexerRps = 0.0;
  private boolean feederReversing = false;
  private double feederReverseEndTimestamp = 0.0;
  private final Debouncer feederJamDebouncer =
      new Debouncer(IndexerConstants.FeederJamDebounceSec, Debouncer.DebounceType.kRising);

  private IndexerSubsystem() {
    if (Constants.currentMode == Constants.Mode.REAL) {
      io = new IndexerIOPhoenix6();
    } else if (Constants.currentMode == Constants.Mode.SIM) {
      io = new IndexerIOSim();
    } else {
      io = new IndexerIO() {};
    }
  }

  public void feed() {
    feeding = true;
    setVelocities(IndexerConstants.FeedRps, IndexerConstants.IndexerRps);
  }

  public void stop() {
    feeding = false;
    setVelocities(0.0, 0.0);
  }

  public void setVelocities(double feederRps, double indexerRps) {
    requestedFeederRps = feederRps;
    requestedIndexerRps = indexerRps;
  }

  public boolean isFeeding() {
    return feeding;
  }

  public double getFeederSupplyCurrentA() {
    return inputs.feederSupplyCurrentA + inputs.feederFollowerSupplyCurrentA;
  }

  public double getTotalSupplyCurrentA() {
    return getFeederSupplyCurrentA()
        + inputs.indexerSupplyCurrentA
        + inputs.indexerFollowerSupplyCurrentA;
  }

  public double getTotalPowerW() {
    return inputs.feederSupplyVoltageV * inputs.feederSupplyCurrentA
        + inputs.feederFollowerSupplyVoltageV * inputs.feederFollowerSupplyCurrentA
        + inputs.indexerSupplyVoltageV * inputs.indexerSupplyCurrentA
        + inputs.indexerFollowerSupplyVoltageV * inputs.indexerFollowerSupplyCurrentA;
  }

  private void applyVelocities() {
    double now = Timer.getFPGATimestamp();

    if (feederReversing) {
      if (now >= feederReverseEndTimestamp) {
        feederReversing = false;
      } else {
        io.setFeederRps(-IndexerConstants.FeedRps);
        io.setIndexerRps(-IndexerConstants.IndexerRps);
        return;
      }
    }

    boolean feederJamInput =
        requestedFeederRps > 0.0
            && getFeederSupplyCurrentA() > IndexerConstants.FeederJamCurrentThresholdA;
    if (feederJamDebouncer.calculate(feederJamInput)) {
      feederReversing = true;
      feederReverseEndTimestamp = now + IndexerConstants.FeederJamReverseDurationSec;
      io.setFeederRps(-IndexerConstants.FeedRps);
      io.setIndexerRps(-IndexerConstants.IndexerRps);
      return;
    }

    io.setFeederRps(requestedFeederRps);
    io.setIndexerRps(requestedIndexerRps);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    applyVelocities();
    Logger.processInputs("Indexer", inputs);
    Logger.recordOutput("Indexer/Feeding", feeding);
    Logger.recordOutput("Indexer/FeederReversing", feederReversing);
    Logger.recordOutput("Indexer/FeederSupplyCurrentA", getFeederSupplyCurrentA());
    Logger.recordOutput("Indexer/IndexerVelocityRPS", inputs.indexerVelocityRps);
    Logger.recordOutput("Indexer/FeederVelocityRPS", inputs.feederVelocityRps);
    Logger.recordOutput("Indexer/TotalSupplyCurrentA", getTotalSupplyCurrentA());
    Logger.recordOutput("Indexer/TotalPowerW", getTotalPowerW());
  }
}
