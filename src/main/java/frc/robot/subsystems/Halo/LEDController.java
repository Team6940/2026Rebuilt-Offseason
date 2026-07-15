// Reference: 694 StuyPulse
package frc.robot.subsystems.Halo;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Ports;
import frc.robot.Constants.Settings;
import frc.robot.subsystems.SuperStructure;
import frc.robot.subsystems.SuperStructure.ChassisMode;
import frc.robot.subsystems.SuperStructure.ShootPhase;
import java.util.ArrayList;
import java.util.List;
import org.littletonrobotics.junction.Logger;

public class LEDController extends SubsystemBase {

  private static LEDController instance;
  private static final int MAX_LAYERS = 3;

  private AddressableLED led;
  private AddressableLEDBuffer buffer;
  private AddressableLEDBufferView chassisLeftView;
  private AddressableLEDBufferView shooterMidView;
  private AddressableLEDBufferView chassisRightView;

  private double attackBandOffset = 0;

  /** A single color layer: spreadColor overtakes baseColor as progress goes 0→1. */
  private static class ColorLayer {
    final LEDPattern baseColor;
    final LEDPattern spreadColor;
    final double speed;
    final double target;
    final boolean reverse; // true = spread from edges inward
    double progress;

    ColorLayer(LEDPattern base, LEDPattern spread, double speed, double target, boolean reverse) {
      this.baseColor = base;
      this.spreadColor = spread;
      this.speed = speed;
      this.target = target;
      this.reverse = reverse;
      this.progress = 0;
    }

    ColorLayer(LEDPattern base, LEDPattern spread, double speed, double target) {
      this(base, spread, speed, target, false);
    }
  }

  private final List<ColorLayer> layers = new ArrayList<>();
  private ShootPhase lastPhase = ShootPhase.OFF;

  public static LEDController getInstance() {
    if (instance == null) {
      instance = new LEDController();
    }
    return instance;
  }

  private LEDController() {
    this.led = new AddressableLED(Ports.LED.LEDPWMPort);
    this.buffer = new AddressableLEDBuffer(Settings.LED.LEDLength);
    led.setLength(buffer.getLength());
    led.setData(buffer);
    led.start();
    this.chassisLeftView =
        buffer.createView(Settings.LED.ChassisLeft[0], Settings.LED.ChassisLeft[1]);
    this.shooterMidView = buffer.createView(Settings.LED.ShooterMid[0], Settings.LED.ShooterMid[1]);
    this.chassisRightView =
        buffer.createView(Settings.LED.ChassisRight[0], Settings.LED.ChassisRight[1]);

    Settings.LEDs.OFF.applyTo(buffer);
  }

  @Override
  public void periodic() {
    var ss = SuperStructure.getInstance();
    ChassisMode chassisMode = ss.getChassisMode();
    ShootPhase shootPhase = ss.getShootPhase();

    if (DriverStation.isDisabled()) {
      Settings.LEDs.DISABLED.breathe(Units.Seconds.of(2)).applyTo(buffer);
    } else {
      // 1. Push new layer on phase transition
      applyPattern(shootPhase, ss);

      // 2. Advance all layers, render bottom-up
      applyAnimation();

      // 3. Chassis sides — only when not in Shooting mode
      if (chassisMode != ChassisMode.SHOOTING) {
        LEDPattern chassisPattern = getChassisModePattern(chassisMode, shootPhase);
        chassisPattern.applyTo(chassisLeftView);
        chassisPattern.applyTo(chassisRightView);
      }
      // ATTACK: overlay rolling band
      if (chassisMode == ChassisMode.ATTACKMODE) {
        applyAttackBand();
      }
    }

    led.setData(buffer);
    Logger.recordOutput("LED/ChassisMode", chassisMode.name());
    Logger.recordOutput("LED/ShootPhase", shootPhase.name());
    Logger.recordOutput("LED/IsDisabled", DriverStation.isDisabled());
    Logger.recordOutput("LED/LayerCount", layers.size());
    publishColorForElastic();
  }

  // ==================== Pattern Declaration ====================

  private void applyPattern(ShootPhase phase, SuperStructure ss) {
    if (phase != lastPhase) {
      pushLayer(phase, ss);
      lastPhase = phase;
    }
  }

  /** Creates and pushes a new ColorLayer based on the target phase. */
  private void pushLayer(ShootPhase phase, SuperStructure ss) {
    boolean fromShoot =
        lastPhase == ShootPhase.AIM
            || lastPhase == ShootPhase.READY
            || lastPhase == ShootPhase.SHOOT;

    ColorLayer newLayer;
    switch (phase) {
      case OFF:
        newLayer = new ColorLayer(Settings.LEDs.OFF, Settings.LEDs.OFF, 0, 1.0);
        break;
      case HEATUP:
        // Reverse spread when coming from a shoot phase — retracts the shoot color
        newLayer = new ColorLayer(Settings.LEDs.OFF, Settings.LEDs.HEATUP, 0.03, 1.0, fromShoot);
        break;
      case AIM:
        newLayer =
            new ColorLayer(
                Settings.LEDs.HEATUP, Settings.LEDs.AIM, 0.04, ss.getAimReadyCount() / 3.0);
        break;
      case READY:
        newLayer = new ColorLayer(Settings.LEDs.AIM, Settings.LEDs.READY, 0.06, 1.0);
        break;
      case SHOOT:
        newLayer = new ColorLayer(Settings.LEDs.READY, Settings.LEDs.SHOOT, 0.08, 1.0);
        break;
      default:
        return;
    }

    layers.add(newLayer);

    // Evict oldest if over limit — oldest becomes the new base (merged into layer below)
    while (layers.size() > MAX_LAYERS) {
      layers.remove(0);
    }
  }

  // ==================== Animation Rendering ====================

  /**
   * Renders all layers bottom-up. Each layer writes baseColor to the full strip, then spreads
   * spreadColor from center. Later layers overwrite earlier ones, so the topmost visible spread
   * wins.
   */
  private void applyAnimation() {
    int start = 0;
    int end = Settings.LED.LEDLength - 1;
    int center = (start + end) / 2; // temporary right shift

    // Reusable buffer for pattern color extraction (avoids per-pixel allocation)
    var tmpBuf = new AddressableLEDBuffer(1);

    for (ColorLayer layer : layers) {
      layer.progress = linearApproach(layer.progress, layer.target, layer.speed);

      // Extract spread color RGB once
      layer.spreadColor.applyTo(tmpBuf);
      Color sc = tmpBuf.getLED(0);
      int sr = (int) (sc.red * 255);
      int sg = (int) (sc.green * 255);
      int sb = (int) (sc.blue * 255);

      double eased = Math.sqrt(layer.progress);

      if (layer.reverse) {
        // Reverse: only write spreadColor to edges, let previous layers show in center
        int safeHalf = (int) ((1.0 - eased) * Math.min(center - start, end - center));
        for (int i = start; i <= end; i++) {
          if (i <= center - safeHalf || i >= center + safeHalf) {
            buffer.setRGB(i, sr, sg, sb);
          }
        }
      } else {
        // Normal: write baseColor to full strip, then spread from center
        layer.baseColor.applyTo(buffer);

        int leftDist = center - start;
        int leftRadius = (int) (eased * leftDist);
        for (int i = 0; i <= leftRadius; i++) {
          buffer.setRGB(center - i, sr, sg, sb);
        }

        int rightDist = end - center;
        int rightRadius = (int) (eased * rightDist);
        for (int i = 0; i <= rightRadius; i++) {
          buffer.setRGB(center + i, sr, sg, sb);
        }
      }
    }

    if (layers.isEmpty()) {
      Settings.LEDs.OFF.applyTo(buffer);
    }
  }

  private double linearApproach(double current, double target, double speed) {
    if (speed <= 0) return target;
    if (current < target) return Math.min(current + speed, target);
    if (current > target) return Math.max(current - speed, target);
    return current;
  }

  // ==================== Helpers ====================

  private LEDPattern getChassisModePattern(ChassisMode mode, ShootPhase phase) {
    return switch (mode) {
      case NORMAL -> Settings.LEDs.CHASSIS_NORMAL;
      case SHOOTING -> getPatternForPhase(phase);
      case ATTACKMODE -> Settings.LEDs.CHASSIS_ATTACKMODE;
    };
  }

  private LEDPattern getPatternForPhase(ShootPhase phase) {
    return switch (phase) {
      case OFF -> Settings.LEDs.OFF;
      case HEATUP -> Settings.LEDs.HEATUP;
      case AIM -> Settings.LEDs.AIM;
      case READY -> Settings.LEDs.READY;
      case SHOOT -> Settings.LEDs.SHOOT;
    };
  }

  // ==================== Attack Band ====================

  private void applyAttackBand() {
    int midStart = Settings.LED.ShooterMid[0];
    int midEnd = Settings.LED.ShooterMid[1];
    int midLength = midEnd - midStart + 1;
    int halfLength = midLength / 2;
    int center = midStart + halfLength;
    int bandWidth = 3;
    int offset = (int) attackBandOffset;

    int aR = (int) (Settings.LED.AttackModeColor.red * 255);
    int aG = (int) (Settings.LED.AttackModeColor.green * 255);
    int aB = (int) (Settings.LED.AttackModeColor.blue * 255);

    for (int i = 0; i < bandWidth; i++) {
      int li = center - offset - i;
      if (li >= midStart && li <= midEnd) buffer.setRGB(li, aR, aG, aB);
      int ri = center + offset + i;
      if (ri >= midStart && ri <= midEnd) buffer.setRGB(ri, aR, aG, aB);
    }

    attackBandOffset += 0.3;
    if (attackBandOffset > halfLength) attackBandOffset = 0;
  }

  private void publishColorForElastic() {
    int len = buffer.getLength();
    for (int i = 0; i < len; i++) {
      Color c = buffer.getLED(i);
      SmartDashboard.putString("LED/LED" + i, c.toHexString());
    }
  }
}
