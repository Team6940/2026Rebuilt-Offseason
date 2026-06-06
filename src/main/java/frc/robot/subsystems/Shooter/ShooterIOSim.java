package frc.robot.subsystems.Shooter;

import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.Constants.ShooterConstants;
import frc.robot.util.PhoenixUtil;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.motorsims.MapleMotorSim;
import org.ironmaple.simulation.motorsims.SimMotorConfigs;

/** Physics sim for dual shooter flywheel pairs. */
public class ShooterIOSim extends ShooterIOPhoenix6 {
  private final MapleMotorSim leaderASimulation;
  private final MapleMotorSim leaderBSimulation;

  public ShooterIOSim() {
    leaderASimulation =
        new MapleMotorSim(
            new SimMotorConfigs(
                DCMotor.getKrakenX60Foc(2),
                ShooterConstants.ShooterRatio,
                KilogramSquareMeters.of(0.004),
                Volts.of(0.05)));
    leaderASimulation.useMotorController(new PhoenixUtil.TalonFXMotorControllerSim(leaderA));

    leaderBSimulation =
        new MapleMotorSim(
            new SimMotorConfigs(
                DCMotor.getKrakenX60Foc(2),
                ShooterConstants.ShooterRatio,
                KilogramSquareMeters.of(0.004),
                Volts.of(0.05)));
    leaderBSimulation.useMotorController(new PhoenixUtil.TalonFXMotorControllerSim(leaderB));
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    var dt = SimulatedArena.getSimulationDt();
    leaderASimulation.update(dt);
    leaderBSimulation.update(dt);
    super.updateInputs(inputs);
  }
}
