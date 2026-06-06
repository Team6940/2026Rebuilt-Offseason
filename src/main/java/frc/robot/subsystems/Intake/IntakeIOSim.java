package frc.robot.subsystems.Intake;

import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.IntakeConstants;
import frc.robot.util.PhoenixUtil;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.motorsims.MapleMotorSim;
import org.ironmaple.simulation.motorsims.SimMotorConfigs;

/** Physics sim for intake rack (position) and rollers (velocity). */
public class IntakeIOSim extends IntakeIOPhoenix6 {
  private final MapleMotorSim rackSimulation;
  private final MapleMotorSim rollerSimulation;

  public IntakeIOSim() {
    rackSimulation =
        new MapleMotorSim(
            new SimMotorConfigs(
                    DCMotor.getKrakenX60Foc(1),
                    IntakeConstants.RackRatio,
                    KilogramSquareMeters.of(0.02),
                    Volts.of(0.1))
                .withHardLimits(
                    Radians.of(Units.rotationsToRadians(IntakeConstants.RackMaxRotations)),
                    Radians.of(Units.rotationsToRadians(IntakeConstants.RackMinRotations))));
    rackSimulation.useMotorController(new PhoenixUtil.TalonFXMotorControllerSim(rackMotor));

    rollerSimulation =
        new MapleMotorSim(
            new SimMotorConfigs(
                DCMotor.getKrakenX60Foc(2),
                IntakeConstants.RollerRatio,
                KilogramSquareMeters.of(0.002),
                Volts.of(0.05)));
    rollerSimulation.useMotorController(new PhoenixUtil.TalonFXMotorControllerSim(rollerLeader));
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    var dt = SimulatedArena.getSimulationDt();
    rackSimulation.update(dt);
    rollerSimulation.update(dt);
    super.updateInputs(inputs);
  }
}
