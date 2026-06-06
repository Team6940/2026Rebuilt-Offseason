package frc.robot.subsystems.Indexer;

import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.Constants.IndexerConstants;
import frc.robot.util.PhoenixUtil;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.motorsims.MapleMotorSim;
import org.ironmaple.simulation.motorsims.SimMotorConfigs;

/** Physics sim for feeder and indexer velocity rollers. */
public class IndexerIOSim extends IndexerIOPhoenix6 {
  private final MapleMotorSim feederSimulation;
  private final MapleMotorSim indexerSimulation;

  public IndexerIOSim() {
    feederSimulation =
        new MapleMotorSim(
            new SimMotorConfigs(
                DCMotor.getKrakenX60Foc(2),
                IndexerConstants.FeederRatio,
                KilogramSquareMeters.of(0.002),
                Volts.of(0.05)));
    feederSimulation.useMotorController(new PhoenixUtil.TalonFXMotorControllerSim(feederLeader));

    indexerSimulation =
        new MapleMotorSim(
            new SimMotorConfigs(
                DCMotor.getKrakenX60Foc(2),
                IndexerConstants.IndexerRatio,
                KilogramSquareMeters.of(0.002),
                Volts.of(0.05)));
    indexerSimulation.useMotorController(new PhoenixUtil.TalonFXMotorControllerSim(indexerLeader));
  }

  @Override
  public void updateInputs(IndexerIOInputs inputs) {
    var dt = SimulatedArena.getSimulationDt();
    feederSimulation.update(dt);
    indexerSimulation.update(dt);
    super.updateInputs(inputs);
  }
}
