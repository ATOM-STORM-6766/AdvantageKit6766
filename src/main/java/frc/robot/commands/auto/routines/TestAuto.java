package frc.robot.commands.auto.routines;

import choreo.auto.AutoFactory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.ChoreoTraj;
import frc.robot.RobotContainer;
import frc.robot.commands.AutoDriveCommands;
import frc.robot.commands.FollowTrajectory;

public final class TestAuto {
  private TestAuto() {}

  public static Command create(
      AutoFactory autoFactory, RobotContainer container, boolean shouldMirror) {
    var drive = container.getDrive();
    var trj = ChoreoTraj.ALL_TRAJECTORIES.get("test");
    return Commands.sequence(
        AutoDriveCommands.resetOdometry(drive, trj.initialPoseBlue(), shouldMirror),
        FollowTrajectory.createOptional(autoFactory, drive, trj, shouldMirror));
  }
}
