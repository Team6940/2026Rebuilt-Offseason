package frc.robot.subsystems.Power;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Hood.HoodSubsystem;
import frc.robot.subsystems.Indexer.IndexerSubsystem;
import frc.robot.subsystems.Intake.IntakeSubsystem;
import frc.robot.subsystems.Shooter.ShooterSubsystem;
import org.littletonrobotics.junction.Logger;

/**
 * Centralized power monitoring subsystem. Aggregates per-subsystem current and power draw, and logs
 * total robot power to AdvantageKit.
 */
public class PowerMonitor extends SubsystemBase {
  private static PowerMonitor instance;

  public static PowerMonitor getInstance() {
    if (instance == null) {
      instance = new PowerMonitor();
    }
    return instance;
  }

  private final ShooterSubsystem shooter;
  private final IntakeSubsystem intake;
  private final HoodSubsystem hood;
  private final IndexerSubsystem indexer;

  private double totalCurrentA = 0.0;
  private double totalPowerW = 0.0;
  private double batteryVoltageV = 12.0;

  private PowerMonitor() {
    shooter = ShooterSubsystem.getInstance();
    intake = IntakeSubsystem.getInstance();
    hood = HoodSubsystem.getInstance();
    indexer = IndexerSubsystem.getInstance();
  }

  public double getTotalCurrentA() {
    return totalCurrentA;
  }

  public double getTotalPowerW() {
    return totalPowerW;
  }

  public double getBatteryVoltageV() {
    return batteryVoltageV;
  }

  @Override
  public void periodic() {
    batteryVoltageV = RobotController.getBatteryVoltage();

    double shooterCurrent = shooter.getTotalSupplyCurrentA();
    double intakeCurrent = intake.getTotalSupplyCurrentA();
    double hoodCurrent = hood.getSupplyCurrentA();
    double indexerCurrent = indexer.getTotalSupplyCurrentA();

    double shooterPower = shooter.getTotalPowerW();
    double intakePower = intake.getTotalPowerW();
    double hoodPower = hood.getPowerW();
    double indexerPower = indexer.getTotalPowerW();

    // Chassis power is logged directly by CommandSwerveDrivetrain; we read from Logger outputs
    // is not possible, so we sum subsystem-only power here for the non-drive breakdown.
    totalCurrentA = shooterCurrent + intakeCurrent + hoodCurrent + indexerCurrent;
    totalPowerW = shooterPower + intakePower + hoodPower + indexerPower;

    Logger.recordOutput("Power/BatteryVoltageV", batteryVoltageV);
    Logger.recordOutput("Power/Subsystems/TotalCurrentA", totalCurrentA);
    Logger.recordOutput("Power/Subsystems/TotalPowerW", totalPowerW);

    Logger.recordOutput("Power/Subsystems/ShooterCurrentA", shooterCurrent);
    Logger.recordOutput("Power/Subsystems/ShooterPowerW", shooterPower);
    Logger.recordOutput("Power/Subsystems/IntakeCurrentA", intakeCurrent);
    Logger.recordOutput("Power/Subsystems/IntakePowerW", intakePower);
    Logger.recordOutput("Power/Subsystems/HoodCurrentA", hoodCurrent);
    Logger.recordOutput("Power/Subsystems/HoodPowerW", hoodPower);
    Logger.recordOutput("Power/Subsystems/IndexerCurrentA", indexerCurrent);
    Logger.recordOutput("Power/Subsystems/IndexerPowerW", indexerPower);
  }
}
