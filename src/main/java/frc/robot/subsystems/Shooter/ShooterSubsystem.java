package frc.robot.subsystems.Shooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

/** Dual leader/follower shooter pairs running at a common target velocity. */
public class ShooterSubsystem extends SubsystemBase {
  private static ShooterSubsystem instance;

  public static ShooterSubsystem getInstance() {
    if (instance == null) {
      instance = new ShooterSubsystem();
    }
    return instance;
  }

  private final ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

  private double targetRps = 0.0;

  private ShooterSubsystem() {
    if (Constants.currentMode == Constants.Mode.REAL) {
      io = new ShooterIOPhoenix6();
    } else if (Constants.currentMode == Constants.Mode.SIM) {
      io = new ShooterIOSim();
    } else {
      io = new ShooterIO() {};
    }
  }

  public void setVelocityRps(double rps) {
    targetRps = rps;
    io.setRps(rps);
  }

  public void stop() {
    setVelocityRps(0.0);
  }

  /** Sim-only: launch one FUEL from the simulated intake. */
  public boolean simulateLaunch(double hoodDegs) {
    return io.simulateLaunch(hoodDegs, targetRps);
  }

  public double getTargetRps() {
    return targetRps;
  }

  public double getVelocityRps() {
    return (inputs.leaderAVelocityRps + inputs.leaderBVelocityRps) / 2.0;
  }

  public double getTotalSupplyCurrentA() {
    return inputs.leaderASupplyCurrentA + inputs.leaderBSupplyCurrentA;
  }

  public double getTotalPowerW() {
    return inputs.leaderASupplyVoltageV * inputs.leaderASupplyCurrentA
        + inputs.leaderBSupplyVoltageV * inputs.leaderBSupplyCurrentA;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter", inputs);
    Logger.recordOutput("Shooter/TargetRps", targetRps);
    Logger.recordOutput("Shooter/VelocityRps", getVelocityRps());
    Logger.recordOutput("Shooter/TotalSupplyCurrentA", getTotalSupplyCurrentA());
    Logger.recordOutput("Shooter/TotalPowerW", getTotalPowerW());
  }
}
