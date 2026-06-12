package frc.robot.subsystems.Shooter;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.simulation.FieldSimulation;

/** Ideal shooter sim: first-order spin-up/down toward commanded RPS (no maple-sim oscillation). */
public class ShooterIOSim implements ShooterIO {
  private double targetRps = 0.0;
  private double velocityRps = 0.0;
  private double lastUpdateSec = Timer.getFPGATimestamp();

  @Override
  public void setRps(double rps) {
    targetRps = rps;
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    double now = Timer.getFPGATimestamp();
    double dt = now - lastUpdateSec;
    lastUpdateSec = now;
    if (dt <= 0.0 || dt > 0.5) {
      dt = 0.02;
    }

    double alpha = dt / ShooterConstants.SimSpinupTimeConstantSec;
    alpha = Math.min(alpha, 1.0);
    velocityRps += (targetRps - velocityRps) * alpha;

    inputs.leaderAConnected = true;
    inputs.leaderBConnected = true;
    inputs.leaderAVelocityRps = velocityRps;
    inputs.leaderBVelocityRps = velocityRps;
  }

  @Override
  public boolean simulateLaunch(double hoodDegs, double shooterRps) {
    if (Constants.currentMode == Constants.Mode.SIM) {
      return FieldSimulation.getInstance().tryLaunchFuel(hoodDegs, velocityRps);
    }
    return false;
  }
}
