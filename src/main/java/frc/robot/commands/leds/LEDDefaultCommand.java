/************************* PROJECT RON *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package frc.robot.commands.leds;

import frc.robot.Constants.Settings;
import frc.robot.Constants.Ports.LED;
import frc.robot.subsystems.leds.LEDController;
import frc.robot.subsystems.SuperStructure;
import java.util.Map;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj2.command.Command;

public class LEDDefaultCommand extends Command {

    private final LEDController leds;

    private final SuperStructure superStructure;


    public LEDDefaultCommand(LEDController leds) {
        this.leds = leds;
        superStructure = SuperStructure.getInstance();
        addRequirements(leds);    
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }

    @Override
    public void execute() {
        if (DriverStation.isDisabled()) {
            LEDPattern base = Settings.LEDs.DISABLED;
            LEDPattern pattern = base.breathe(Units.Seconds.of(2));
            leds.applyPattern(pattern);
            return;
        }
        else{
        // These probably won't actually be what we want the LEDs to be showing
        // TODO: Figure out what we want the LEDs to show
        switch (superStructure.getDriveMode()) {
            case HYBRID_TRENCH:
            {
                leds.applyGyro( Settings.LEDs.HYBRID_TRENCH);
            }
            case HYBRID_INTAKE_DRIVE:
            {
                leds.applyGyro( Settings.LEDs.HYBRID_INTAKE_DRIVE);
            }
            case MANUAL:
            {
                leds.applyGyro( Settings.LEDs.MANUAL);
            }
            case AUTO_AIM:
            {
                leds.applyGyro( Settings.LEDs.MANUAL);

            }
            default:
            {
                leds.applyGyro( Settings.LEDs.DISABLED);
            }

        }
        switch (superStructure.getShootPhase()) {
            case READY:
            {
                leds.applyShoot( Settings.LEDs.READY);
            }
            case SHOOT:
            {
                leds.applyShoot( Settings.LEDs.SHOOT);
            }
            case AIM :
            {
                leds.applyShoot( Settings.LEDs.AIM);
            }
            
        
            default:
            {
                leds.applyShoot(Settings.LEDs.DISABLED);
            }
                
        }
    }
    }
}
