package frc.robot.commands;

import choreo.auto.AutoFactory;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.ChoreoTraj;
import frc.robot.subsystems.drive.Drive;

public class FollowTrajectory {
  private FollowTrajectory() {}

  public static Command create(
      AutoFactory autoFactory, Drive drive, ChoreoTraj trj, int segment, boolean shouldMirror) {
    return autoFactory
        .trajectoryCmd(trj.name(), segment, shouldMirror ? AutoTrajectory::mirrorY : t -> t)
        .andThen(
            AutoDriveCommands.followPoint(drive, trj.segment(segment).endPoseBlue(), shouldMirror))
        .withName("Follow Trajectory: " + trj.name());
  }
}
