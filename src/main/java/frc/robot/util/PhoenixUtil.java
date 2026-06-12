package frc.robot.util;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import java.util.function.Supplier;
import org.ironmaple.simulation.motorsims.SimulatedBattery;
import org.ironmaple.simulation.motorsims.SimulatedMotorController;

public final class PhoenixUtil {
  private PhoenixUtil() {}

  /** Attempts to run the command until no error is produced. */
  public static void tryUntilOk(int maxAttempts, Supplier<StatusCode> command) {
    for (int i = 0; i < maxAttempts; i++) {
      if (command.get().isOK()) {
        break;
      }
    }
  }

  public static class TalonFXMotorControllerSim implements SimulatedMotorController {
    private static int instances = 0;
    public final int id;

    private final TalonFXSimState talonFXSimState;

    public TalonFXMotorControllerSim(TalonFX talonFX) {
      this.id = instances++;
      this.talonFXSimState = talonFX.getSimState();
    }

    @Override
    public Voltage updateControlSignal(
        Angle mechanismAngle,
        AngularVelocity mechanismVelocity,
        Angle encoderAngle,
        AngularVelocity encoderVelocity) {
      talonFXSimState.setRawRotorPosition(encoderAngle);
      talonFXSimState.setRotorVelocity(encoderVelocity);
      talonFXSimState.setSupplyVoltage(SimulatedBattery.getBatteryVoltage());
      return talonFXSimState.getMotorVoltageMeasure();
    }
  }
}
