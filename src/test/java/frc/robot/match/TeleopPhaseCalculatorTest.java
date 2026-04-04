package frc.robot.match;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

class TeleopPhaseCalculatorTest {
  @Test
  void returnsIdleStateWhenTimerHasNotStarted() {
    TeleopPhaseSnapshot snapshot = TeleopPhaseCalculator.calculate(0.0, false, true);

    assertFalse(snapshot.isOurShiftActive());
    assertEquals(0.0, snapshot.currentPhaseTimeRemainingSeconds());
    assertEquals(140.0, snapshot.teleopTimeRemainingSeconds());
  }

  @Test
  void keepsStartingAllianceActiveDuringShiftOneOutsideBlinkWindow() {
    TeleopPhaseSnapshot snapshot = TeleopPhaseCalculator.calculate(20.0, true, true);

    assertTrue(snapshot.isOurShiftActive());
    assertEquals(15.0, snapshot.currentPhaseTimeRemainingSeconds());
    assertEquals(120.0, snapshot.teleopTimeRemainingSeconds());
  }

  @Test
  void invertsInsideBlinkWindowBeforeEndgame() {
    TeleopPhaseSnapshot snapshot = TeleopPhaseCalculator.calculate(29.1, true, true);

    assertFalse(snapshot.isOurShiftActive());
    assertEquals(5.900000000000006, snapshot.currentPhaseTimeRemainingSeconds());
    assertEquals(110.9, snapshot.teleopTimeRemainingSeconds());
  }

  @Test
  void keepsBothActiveDuringEndgameWithoutBlinking() {
    TeleopPhaseSnapshot snapshot = TeleopPhaseCalculator.calculate(115.1, true, false);

    assertTrue(snapshot.isOurShiftActive());
    assertEquals(24.900000000000006, snapshot.currentPhaseTimeRemainingSeconds());
    assertEquals(24.900000000000006, snapshot.teleopTimeRemainingSeconds());
  }
}
