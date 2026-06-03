package frc.robot.subsystems.Shooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.MotorIDs;
import frc.robot.Constants.ShooterConstants;
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

  private final TalonFX leaderA;
  private final TalonFX followerA;
  private final TalonFX leaderB;
  private final TalonFX followerB;
  private final VelocityVoltage velocityRequest = new VelocityVoltage(0.0).withEnableFOC(true);

  private double targetRps = 0.0;
  private double velocityRps = 0.0;

  private ShooterSubsystem() {
    if (Constants.currentMode == Constants.Mode.REAL) {
      leaderA = new TalonFX(MotorIDs.kShooterLeaderMotorIdA, CANBus.roboRIO());
      followerA = new TalonFX(MotorIDs.kShooterFollowerMotorIdA, CANBus.roboRIO());
      leaderB = new TalonFX(MotorIDs.kShooterLeaderMotorIdB, CANBus.roboRIO());
      followerB = new TalonFX(MotorIDs.kShooterFollowerMotorIdB, CANBus.roboRIO());
      configureLeader(leaderA);
      configureLeader(leaderB);
      configureFollower(followerA);
      configureFollower(followerB);
      followerA.setControl(new Follower(leaderA.getDeviceID(), ShooterConstants.FollowerAlignment));
      followerB.setControl(new Follower(leaderB.getDeviceID(), ShooterConstants.FollowerAlignment));
    } else {
      leaderA = null;
      followerA = null;
      leaderB = null;
      followerB = null;
    }
  }

  private void configureLeader(TalonFX motor) {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.Feedback.SensorToMechanismRatio = ShooterConstants.ShooterRatio;
    config.Slot0.kP = ShooterConstants.kP;
    config.Slot0.kI = ShooterConstants.kI;
    config.Slot0.kD = ShooterConstants.kD;
    config.Slot0.kV = ShooterConstants.kV;
    config.Slot0.kS = ShooterConstants.kS;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = ShooterConstants.SupplyCurrentLimit;
    config.CurrentLimits.StatorCurrentLimit = ShooterConstants.StatorCurrentLimit;
    config.MotorOutput.Inverted = ShooterConstants.Inverted;
    motor.getConfigurator().apply(config);
  }

  private void configureFollower(TalonFX motor) {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.MotorOutput.Inverted = ShooterConstants.Inverted;
    motor.getConfigurator().apply(config);
  }

  public void setVelocityRps(double rps) {
    targetRps = rps;
    if (leaderA == null) {
      velocityRps = rps;
      return;
    }
    if (rps == 0.0) {
      leaderA.stopMotor();
      leaderB.stopMotor();
    } else {
      leaderA.setControl(velocityRequest.withVelocity(rps));
      leaderB.setControl(velocityRequest.withVelocity(rps));
    }
  }

  public void stop() {
    setVelocityRps(0.0);
  }

  public double getTargetRps() {
    return targetRps;
  }

  public double getVelocityRps() {
    return velocityRps;
  }

  @Override
  public void periodic() {
    if (leaderA != null) {
      BaseStatusSignal.refreshAll(leaderA.getVelocity(), leaderB.getVelocity());
      velocityRps =
          (leaderA.getVelocity().getValueAsDouble() + leaderB.getVelocity().getValueAsDouble())
              / 2.0;
    }

    Logger.recordOutput("Shooter/TargetRps", targetRps);
    Logger.recordOutput("Shooter/VelocityRps", velocityRps);
  }
}
