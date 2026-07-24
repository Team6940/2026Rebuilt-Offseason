package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Indexer.IndexerSubsystem;
import org.littletonrobotics.junction.Logger;

public class GamePeriodReminder extends SubsystemBase {
  private static final int NUM_LEDS = 5;
  private static final double YELLOW_THRESHOLD = 15.0;
  private static final double RED_COUNTDOWN_THRESHOLD = 5.0;
  private static final double TOTAL_MATCH_TIME = 150.0;

  /** Match time remaining (s) at which endgame low-power current limits engage. */
  public static final double ENDGAME_LOW_POWER_THRESHOLD_SEC = 50.0;

  private static GamePeriodReminder instance;

  private boolean endgameLowPowerApplied = false;

  private GamePeriodReminder() {}

  public static GamePeriodReminder getInstance() {
    if (instance == null) {
      instance = new GamePeriodReminder();
    }
    return instance;
  }

  /**
   * Approximate match time remaining from the DS/FMS. During teleop this counts down; returns a
   * negative value when unavailable.
   */
  public double getMatchTimeRemaining() {
    return DriverStation.getMatchTime();
  }

  /**
   * True during teleop when remaining match time is at or below {@link
   * #ENDGAME_LOW_POWER_THRESHOLD_SEC}.
   */
  public boolean isEndgameLowPower() {
    if (!DriverStation.isTeleopEnabled()) {
      return false;
    }
    double matchTime = getMatchTimeRemaining();
    return matchTime >= 0.0 && matchTime <= ENDGAME_LOW_POWER_THRESHOLD_SEC;
  }

  @Override
  public void periodic() {
    String[] colors = new String[NUM_LEDS];
    String state;

    if (DriverStation.isDisabled()) {
      state = "DISABLED";
      for (int i = 0; i < NUM_LEDS; i++) {
        colors[i] = "#000000";
      }
    } else if (DriverStation.isAutonomousEnabled()) {
      state = "AUTO";
      for (int i = 0; i < NUM_LEDS; i++) {
        colors[i] = "#00FF00";
      }
    } else if (DriverStation.isTeleopEnabled()) {
      double matchTime = DriverStation.getMatchTime();
      boolean ourHubActive = isOurHubActive(matchTime);
      double secondsRemainingInShift = getSecondsRemainingInShift(matchTime);

      if (!ourHubActive && secondsRemainingInShift > YELLOW_THRESHOLD) {
        state = "INACTIVE";
        for (int i = 0; i < NUM_LEDS; i++) {
          colors[i] = "#000000";
        }
      } else if (secondsRemainingInShift <= RED_COUNTDOWN_THRESHOLD) {
        state = "RED_COUNTDOWN";
        int litCount = (int) (RED_COUNTDOWN_THRESHOLD - secondsRemainingInShift) + 1;
        for (int i = 0; i < NUM_LEDS; i++) {
          colors[i] = i < litCount ? "#FF0000" : "#000000";
        }
      } else if (secondsRemainingInShift <= YELLOW_THRESHOLD) {
        state = "YELLOW_WARNING";
        for (int i = 0; i < NUM_LEDS; i++) {
          colors[i] = "#FFFF00";
        }
      } else {
        state = "ACTIVE";
        for (int i = 0; i < NUM_LEDS; i++) {
          colors[i] = "#00FF00";
        }
      }
    } else {
      state = "IDLE";
      for (int i = 0; i < NUM_LEDS; i++) {
        colors[i] = "#000000";
      }
    }

    for (int i = 0; i < NUM_LEDS; i++) {
      SmartDashboard.putString("GamePeriodReminder/LED" + i, colors[i]);
    }
    SmartDashboard.putString("GamePeriodReminder/State", state);

    updateEndgameLowPowerLimits();
  }

  /**
   * On first entry into endgame: lower feeder/indexer stator limits, and re-apply chassis limits for
   * the current {@link SuperStructure.ChassisMode} (so NORMAL/ATTACKMODE drop to Aim limits without
   * disturbing SHOOTING mode state).
   */
  private void updateEndgameLowPowerLimits() {
    boolean endgame = isEndgameLowPower();
    Logger.recordOutput("GamePeriodReminder/MatchTimeRemaining", getMatchTimeRemaining());
    Logger.recordOutput("GamePeriodReminder/EndgameLowPower", endgame);
    Logger.recordOutput("GamePeriodReminder/EndgameLowPowerApplied", endgameLowPowerApplied);

    if (!endgame || endgameLowPowerApplied) {
      return;
    }
    endgameLowPowerApplied = true;

    IndexerSubsystem.getInstance().applyStatorCurrentLimitLow();

    // Re-apply limits for the active mode. Do not call setChassisMode — that would early-return
    // when already NORMAL/ATTACKMODE, and must not rewrite previousChassisMode while SHOOTING.
    SuperStructure superStructure = SuperStructure.getInstance();
    superStructure.getDrive().applyChassisModeLimits(superStructure.getChassisMode());
  }

  private boolean isOurHubActive(double matchTime) {
    var alliance = DriverStation.getAlliance();
    if (alliance.isEmpty()) return true;

    if (matchTime > 130) {
      return true;
    }

    String gameData = DriverStation.getGameSpecificMessage();
    if (gameData == null) return true;
    if (gameData.isEmpty()) return true;
    if (gameData.charAt(0) != 'R' || gameData.charAt(0) != 'B' ) return true;
    boolean redInactiveFirst = gameData.charAt(0) == 'R';

    boolean shift1Active;
    if (alliance.get() == edu.wpi.first.wpilibj.DriverStation.Alliance.Red) {
      shift1Active = !redInactiveFirst;
    } else {
      shift1Active = redInactiveFirst;
    }

    if (matchTime > 105) {
      return shift1Active;
    } else if (matchTime > 80) {
      return !shift1Active;
    } else if (matchTime > 55) {
      return shift1Active;
    } else if (matchTime > 30) {
      return !shift1Active;
    } else {
      return true;
    }
  }

  private double getSecondsRemainingInShift(double matchTime) {
    if (matchTime > 130) return matchTime - 130;
    if (matchTime > 105) return matchTime - 105;
    if (matchTime > 80) return matchTime - 80;
    if (matchTime > 55) return matchTime - 55;
    if (matchTime > 30) return matchTime - 30;
    return matchTime;
  }
}
