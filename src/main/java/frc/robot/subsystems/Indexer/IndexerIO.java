package frc.robot.subsystems.Indexer;

import org.littletonrobotics.junction.AutoLog;

public interface IndexerIO {
  @AutoLog
  public class IndexerIOInputs {
    public boolean feederConnected = false;
    public double feederVelocityRps = 0.0;
    public double feederSupplyVoltageV = 0.0;
    public double feederSupplyCurrentA = 0.0;
    public boolean indexerConnected = false;
    public double indexerVelocityRps = 0.0;
    public double indexerSupplyVoltageV = 0.0;
    public double indexerSupplyCurrentA = 0.0;
    public boolean feederFollowerConnected = false;
    public double feederFollowerSupplyVoltageV = 0.0;
    public double feederFollowerSupplyCurrentA = 0.0;
    public boolean indexerFollowerConnected = false;
    public double indexerFollowerSupplyVoltageV = 0.0;
    public double indexerFollowerSupplyCurrentA = 0.0;
  }

  public default void setFeederRps(double rps) {}

  public default void setIndexerRps(double rps) {}

  public default void updateInputs(IndexerIOInputs inputs) {}
}
