package frc.robot.match;

public final class TeleopPhaseCalculator {
  public static final double TELEOP_DURATION_SECONDS = 140.0;
  private static final double BLINK_WINDOW_SECONDS = 7.0;
  private static final double BLINK_PERIOD_SECONDS = 0.5;
  private static final double BLINK_ACTIVE_SECONDS = 0.25;
  private static final double ENDGAME_DURATION_SECONDS = 30.0;

  private TeleopPhaseCalculator() {}

  public static TeleopPhaseSnapshot calculate(
      double elapsedTeleopSeconds, boolean timerRunning, boolean startsActive) {
    if (elapsedTeleopSeconds == 0.0 && !timerRunning) {
      return new TeleopPhaseSnapshot(false, 0.0, TELEOP_DURATION_SECONDS);
    }

    double teleopTimeRemainingSeconds =
        Math.max(0.0, TELEOP_DURATION_SECONDS - elapsedTeleopSeconds);
    boolean isOurShiftActive;
    double currentPhaseTimeRemainingSeconds;

    if (teleopTimeRemainingSeconds > 130.0) {
      isOurShiftActive = true;
      currentPhaseTimeRemainingSeconds = teleopTimeRemainingSeconds - 130.0;
    } else if (teleopTimeRemainingSeconds > 105.0) {
      isOurShiftActive = startsActive;
      currentPhaseTimeRemainingSeconds = teleopTimeRemainingSeconds - 105.0;
    } else if (teleopTimeRemainingSeconds > 80.0) {
      isOurShiftActive = !startsActive;
      currentPhaseTimeRemainingSeconds = teleopTimeRemainingSeconds - 80.0;
    } else if (teleopTimeRemainingSeconds > 55.0) {
      isOurShiftActive = startsActive;
      currentPhaseTimeRemainingSeconds = teleopTimeRemainingSeconds - 55.0;
    } else if (teleopTimeRemainingSeconds > ENDGAME_DURATION_SECONDS) {
      isOurShiftActive = !startsActive;
      currentPhaseTimeRemainingSeconds = teleopTimeRemainingSeconds - ENDGAME_DURATION_SECONDS;
    } else if (teleopTimeRemainingSeconds > 0.0) {
      isOurShiftActive = true;
      currentPhaseTimeRemainingSeconds = teleopTimeRemainingSeconds;
    } else {
      isOurShiftActive = false;
      currentPhaseTimeRemainingSeconds = 0.0;
    }

    boolean isEndgame =
        teleopTimeRemainingSeconds > 0.0 && teleopTimeRemainingSeconds <= ENDGAME_DURATION_SECONDS;
    if (!isEndgame
        && currentPhaseTimeRemainingSeconds > 0.0
        && currentPhaseTimeRemainingSeconds <= BLINK_WINDOW_SECONDS
        && (elapsedTeleopSeconds % BLINK_PERIOD_SECONDS) < BLINK_ACTIVE_SECONDS) {
      isOurShiftActive = !isOurShiftActive;
    }

    return new TeleopPhaseSnapshot(
        isOurShiftActive,
        Math.max(0.0, currentPhaseTimeRemainingSeconds),
        teleopTimeRemainingSeconds);
  }
}
