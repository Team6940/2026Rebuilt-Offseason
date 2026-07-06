/************************* PROJECT RON *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
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
    public boolean isFinished(){
        return false;
    }
}
