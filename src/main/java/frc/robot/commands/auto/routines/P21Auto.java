package frc.robot.commands.auto.routines;

import choreo.auto.AutoFactory;
import edu.wpi.first.wpilibj2.command.Command;

public final class P21Auto {
  private P21Auto() {}

  public static Command create(AutoFactory autoFactory) {
    return autoFactory
        .trajectoryCmd("t5", 0)
        .beforeStarting(autoFactory.resetOdometry("t5"))
        .andThen(autoFactory.trajectoryCmd("t5", 1));
  }
}
