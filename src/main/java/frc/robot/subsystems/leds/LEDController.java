/************************* PROJECT RON *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/
package frc.robot.subsystems.leds;

import frc.robot.Constants.Ports;
import frc.robot.Constants.Settings;
import frc.robot.subsystems.SuperStructure;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class LEDController extends SubsystemBase {

    private static LEDController instance;
    
        private final LEDPattern defaultPattern = Settings.LEDs.DISABLED;
    
        private AddressableLED led;
    
        private AddressableLEDBuffer buffer;
    
        private AddressableLEDBufferView gyroView;

        private AddressableLEDBufferView shooterView;

    private String currentPatternName = "DISABLED";

    public static LEDController getInstance() {
        if (instance == null) {
        instance = new LEDController();
        }
        return instance;
    }

    private LEDController() {
        this.led = new AddressableLED(Ports.LED.LED_PWM_PORT);
        this.buffer = new AddressableLEDBuffer(Settings.LED.LED_LENGTH);
        led.setLength(buffer.getLength());
        led.setData(buffer);
        led.start();
        this.gyroView = buffer.createView(Settings.LED.GYRO_BUFFER[0], Settings.LED.GYRO_BUFFER[1]);
        this.shooterView = buffer.createView(Settings.LED.SHOOTER_BUFFER[0], Settings.LED.SHOOTER_BUFFER[1]);


        applyAll(defaultPattern);
        SmartDashboard.putData(instance);
    }



    public void applyAll(LEDPattern pattern) {
        pattern.applyTo(buffer);
        currentPatternName = "ALL";
    }

    
    public void applyShoot(LEDPattern pattern) {
        pattern.applyTo(buffer);
        currentPatternName = "SHOOT";
    }

    
    public void applyGyro(LEDPattern pattern) {
        pattern.applyTo(gyroView);
        currentPatternName = "GYRO";
    }

    public void applyPattern(LEDPattern pattern) {
        pattern.applyTo(buffer);
        currentPatternName = "PATTERN";
    }

    public void updateFromSuperStructure() {
        var superStructure = SuperStructure.getInstance();
        LEDPattern pattern;
        if (DriverStation.isDisabled()) {
            pattern = Settings.LEDs.DISABLED.breathe(Units.Seconds.of(2));
            currentPatternName = "DISABLED";
        } else {
            pattern =
                switch (superStructure.getShootPhase()) {
                    case READY -> Settings.LEDs.READY;
                    case SHOOT -> Settings.LEDs.SHOOT;
                    case AIM -> Settings.LEDs.AIM;
                    case HEATUP -> Settings.LEDs.HEATUP;
                    case OFF -> Settings.LEDs.OFF;
                };
            currentPatternName = superStructure.getShootPhase().name();
        }

        pattern.applyTo(buffer);
        led.setData(buffer);
    }

    @Override
    public void periodic() {
        updateFromSuperStructure();

        // Log the dynamic state that determines what the LEDs are currently displaying
        var superStructure = SuperStructure.getInstance();
        Logger.recordOutput("LED/DriveMode", superStructure.getDriveMode().name());
        Logger.recordOutput("LED/ShootPhase", superStructure.getShootPhase().name());
        Logger.recordOutput("LED/IsDisabled", DriverStation.isDisabled());
        Logger.recordOutput("LED/CurrentPattern", currentPatternName);
        
        // Log the first LED color as a sample of current output
        if (buffer.getLength() > 0) {
            double r = buffer.getLED(0).red;
            double g = buffer.getLED(0).green;
            double b = buffer.getLED(0).blue;
            Logger.recordOutput("LED/FirstLEDColor", String.format("RGB(%d,%d,%d)", (double)r, (double)g, (double)b));
        }
    }
}
