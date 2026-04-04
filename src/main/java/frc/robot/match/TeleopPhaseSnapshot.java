package frc.robot.match;

public final class TeleopPhaseSnapshot {
  private final boolean isOurShiftActive;
  private final double currentPhaseTimeRemainingSeconds;
  private final double teleopTimeRemainingSeconds;

  public TeleopPhaseSnapshot(
      boolean isOurShiftActive,
      double currentPhaseTimeRemainingSeconds,
      double teleopTimeRemainingSeconds) {
    this.isOurShiftActive = isOurShiftActive;
    this.currentPhaseTimeRemainingSeconds = currentPhaseTimeRemainingSeconds;
    this.teleopTimeRemainingSeconds = teleopTimeRemainingSeconds;
  }

  public boolean isOurShiftActive() {
    return isOurShiftActive;
  }

  public double currentPhaseTimeRemainingSeconds() {
    return currentPhaseTimeRemainingSeconds;
  }

  public double teleopTimeRemainingSeconds() {
    return teleopTimeRemainingSeconds;
  }
}
