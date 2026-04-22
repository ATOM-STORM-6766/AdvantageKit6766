package frc.robot.commands;

import choreo.util.ChoreoAllianceFlipUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.AllianceFlipUtil;

public final class AutoDriveCommands {
  private AutoDriveCommands() {}

  public static Command followPoint(Drive drive, Pose2d targetPose) {
    return followPoint(drive, targetPose, false);
  }

  public static Command followPoint(Drive drive, Pose2d targetPose, boolean shouldMirror) {
    return new FollowPoint(
            drive,
            () ->
                AllianceFlipUtil.apply(
                    shouldMirror
                        ? ChoreoAllianceFlipUtil.getMirrorY().flip(targetPose)
                        : targetPose))
        .withName("Follow Point");
  }

  public static Command zeroChassisSpeeds(Drive drive) {
    return Commands.runOnce(() -> drive.runVelocity(new ChassisSpeeds()), drive)
        .withName("Zero Chassis Speeds");
  }

  public static Command stopDrive(Drive drive) {
    return Commands.runOnce(drive::stop, drive).withName("Stop Drive");
  }

  public static Command resetOdometry(Drive drive, Pose2d pose, boolean shouldMirror) {
    return Commands.runOnce(
            () ->
                drive.setPose(
                    AllianceFlipUtil.apply(
                        shouldMirror ? ChoreoAllianceFlipUtil.getMirrorY().flip(pose) : pose)),
            drive)
        .withName("Reset Odometry");
  }
}
