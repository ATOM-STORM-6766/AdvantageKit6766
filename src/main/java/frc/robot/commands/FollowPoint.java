package frc.robot.commands;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import java.util.function.Supplier;

public class FollowPoint extends Command {
  private static PIDController trans = new PIDController(5, 0, 0); // TODO 移动到constants
  private static ProfiledPIDController rotation =
      new ProfiledPIDController(
          5, 0, 0, new TrapezoidProfile.Constraints(13.200, 36.366)); // TODO 移动到constants
  private static HolonomicDriveController holonomicController =
      new HolonomicDriveController(trans, trans, rotation);

  static {
    holonomicController.setTolerance(new Pose2d(0.02, 0.02, Rotation2d.fromDegrees(1)));
  }

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
        holonomicController.calculate(currentPose, targetPose, 0, targetPose.getRotation());
    m_drive.runVelocity(targetChassisSpeeds);
  }

  @Override
  public void end(boolean interrupted) {
    m_drive.stop();
  }

  @Override
  public boolean isFinished() {
    return holonomicController.atReference();
  }
}
