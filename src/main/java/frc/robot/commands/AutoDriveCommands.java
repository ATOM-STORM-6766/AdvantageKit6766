package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.AllianceFlipUtil;

public final class AutoDriveCommands {
  private AutoDriveCommands() {}

  public static Command followPoint(Drive drive, Pose2d targetPose) {
    return new FollowPoint(drive, () -> AllianceFlipUtil.apply(targetPose))
        .withName("Follow Point");
  }

  public static Command zeroChassisSpeeds(Drive drive) {
    return Commands.runOnce(() -> drive.runVelocity(new ChassisSpeeds()), drive)
        .withName("Zero Chassis Speeds");
  }

  public static Command stopDrive(Drive drive) {
    return Commands.runOnce(drive::stop, drive).withName("Stop Drive");
  }
}
