// Code Reference: 694 StuyPulse
package frc.robot.commands.leds;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.leds.LEDController;

public class LEDDefaultCommand extends Command {

  private final LEDController leds;

  public LEDDefaultCommand(LEDController leds) {
    this.leds = leds;
    addRequirements(leds);
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }

  @Override
  public void execute() {
    leds.updateFromSuperStructure();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
