package frc.robot.subsystems.Hood;

import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.HoodConstants;
import frc.robot.util.PhoenixUtil;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.motorsims.MapleMotorSim;
import org.ironmaple.simulation.motorsims.SimMotorConfigs;

/** Physics sim for hood position control using Phoenix 6 closed-loop + maple-sim. */
public class HoodIOSim extends HoodIOPhoenix6 {
  private final MapleMotorSim simulation;

  public HoodIOSim() {
    simulation =
        new MapleMotorSim(
            new SimMotorConfigs(
                    DCMotor.getKrakenX60Foc(1),
                    HoodConstants.HoodRatio,
                    KilogramSquareMeters.of(0.05),
                    Volts.of(0.1))
                .withHardLimits(
                    Radians.of(Units.degreesToRadians(HoodConstants.MaxDegs)),
                    Radians.of(Units.degreesToRadians(HoodConstants.MinDegs))));
    simulation.useMotorController(new PhoenixUtil.TalonFXMotorControllerSim(motor));
  }

  @Override
  public void updateInputs(HoodIOInputs inputs) {
    simulation.update(SimulatedArena.getSimulationDt());
    super.updateInputs(inputs);
  }
}
