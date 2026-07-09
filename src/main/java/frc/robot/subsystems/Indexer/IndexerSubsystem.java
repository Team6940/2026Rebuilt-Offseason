package frc.robot.subsystems.Indexer;

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

  private void setVelocities(double feederRps, double indexerRps) {
    io.setFeederRps(feederRps);
    io.setIndexerRps(indexerRps);
  }

  public boolean isFeeding() {
    return feeding;
  }

  public double getTotalSupplyCurrentA() {
    return inputs.feederSupplyCurrentA + inputs.indexerSupplyCurrentA;
  }

  public double getTotalPowerW() {
    return inputs.feederSupplyVoltageV * inputs.feederSupplyCurrentA
        + inputs.indexerSupplyVoltageV * inputs.indexerSupplyCurrentA;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Indexer", inputs);
    Logger.recordOutput("Indexer/Feeding", feeding);
    Logger.recordOutput("Indexer/TotalSupplyCurrentA", getTotalSupplyCurrentA());
    Logger.recordOutput("Indexer/TotalPowerW", getTotalPowerW());
  }
}
