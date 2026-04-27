package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Drive;
import java.util.function.Supplier;

public class FollowPoint extends Command {
  private Drive m_drive;
  private Supplier<Pose2d> m_targetPoseSupplier;

  public FollowPoint(Drive drive, Supplier<Pose2d> targetPoseSupplier) {
    m_drive = drive;
    m_targetPoseSupplier = targetPoseSupplier;
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    Pose2d targetPose = m_targetPoseSupplier.get();
    Pose2d currentPose = m_drive.getPose();
    var targetChassisSpeeds =
        Constants.DriveControlConstants.FollowPoint.holonomicController.calculate(
            currentPose, targetPose, 0.1, targetPose.getRotation());
    m_drive.runVelocity(targetChassisSpeeds);
  }

  @Override
  public void end(boolean interrupted) {
    m_drive.stop();
  }

  @Override
  public boolean isFinished() {
    return Constants.DriveControlConstants.FollowPoint.holonomicController.atReference();
  }
}
