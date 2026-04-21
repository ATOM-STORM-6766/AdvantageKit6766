package frc.robot.match;

import edu.wpi.first.wpilibj.Timer;
import org.littletonrobotics.junction.Logger;

public class TeleopPhaseTracker {
  private final Timer timer;

  public TeleopPhaseTracker() {
    this(new Timer());
  }

  TeleopPhaseTracker(Timer timer) {
    this.timer = timer;
  }

  public void start() {
    timer.start();
  }

  public void stopAndReset() {
    timer.stop();
    timer.reset();
  }

  public void log(boolean startsActive) {
    TeleopPhaseSnapshot teleopPhaseSnapshot =
        TeleopPhaseCalculator.calculate(timer.get(), timer.isRunning(), startsActive);

    Logger.recordOutput("Game/IsOurShiftActive", teleopPhaseSnapshot.isOurShiftActive());
    Logger.recordOutput(
        "Game/CurrentShiftTimeRemaining", teleopPhaseSnapshot.currentPhaseTimeRemainingSeconds());
    Logger.recordOutput(
        "Game/TeleopTimeRemaining", teleopPhaseSnapshot.teleopTimeRemainingSeconds());
  }
}
