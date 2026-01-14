package frc.robot.commands;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import java.util.function.Supplier;

public class TargetCommand extends Command {
  private HolonomicDriveController holonomicController;
  private Drive m_drive;
  private Supplier<Pose2d> m_targetPose;

  public TargetCommand(Drive drive, Supplier<Pose2d> targetPose) {
    holonomicController =
        new HolonomicDriveController(
            new PIDController(3, 0, 0),
            new PIDController(3, 0, 0),
            new ProfiledPIDController(5, 0, 0, new TrapezoidProfile.Constraints(6.28, 3.14)));
    addRequirements(drive);
    m_drive = drive;
    m_targetPose = targetPose;
    holonomicController.setTolerance(new Pose2d(0.02, 0.02, Rotation2d.fromDegrees(1)));
  }

  @Override
  public void execute() {
    ChassisSpeeds chassisSpeeds =
        holonomicController.calculate(m_drive.getPose(), m_targetPose.get(), 0, Rotation2d.kZero);
    if (holonomicController.atReference()) {
      m_drive.stop();
    } else {
      m_drive.runVelocity(chassisSpeeds);
    }
  }
}
