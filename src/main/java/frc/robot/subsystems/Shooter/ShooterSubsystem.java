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
    switch (Constants.currentMode) {
      case REAL -> io = new ShooterIOPhoenix6();
      case SIM -> io = new ShooterIOSim();
      default -> io = new ShooterIO() {};
    }
  }

  public void setVelocityRps(double rps) {
    targetRps = rps;
    io.setRps(rps);
  }

  public void stop() {
    setVelocityRps(0.0);
  }

  public double getTargetRps() {
    return targetRps;
  }

  public double getVelocityRps() {
    return (inputs.leaderAVelocityRps + inputs.leaderBVelocityRps) / 2.0;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter", inputs);
    Logger.recordOutput("Shooter/TargetRps", targetRps);
    Logger.recordOutput("Shooter/VelocityRps", getVelocityRps());
  }
}
