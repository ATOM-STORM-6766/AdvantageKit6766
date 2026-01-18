package frc.robot.commands;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import java.util.function.Supplier;

public class TargetCommand extends Command {
  private HolonomicDriveController holonomicController;
  private Drive m_drive;
  private Supplier<Pose2d> m_targetPose;
  private static PIDController trans = new PIDController(3, 0, 0);
  private static ProfiledPIDController rotation =
      new ProfiledPIDController(5, 0, 0, new TrapezoidProfile.Constraints(6.28, 3.14));

  static {
    rotation.enableContinuousInput(-Math.PI, Math.PI);
  }

  private Timer timer = new Timer();
  private boolean isTrajectory = false;
  private Trajectory m_Trajectory;
  private Trajectory o_Trajectory;

  public TargetCommand(Drive drive, Supplier<Pose2d> targetPose) {
    holonomicController = new HolonomicDriveController(trans, trans, rotation);
    addRequirements(drive);
    m_drive = drive;
    m_targetPose = targetPose;
    holonomicController.setTolerance(new Pose2d(0.02, 0.02, Rotation2d.fromDegrees(1)));
  }

  public TargetCommand(Drive drive, Trajectory trajectory) {
    isTrajectory = true;
    holonomicController = new HolonomicDriveController(trans, trans, rotation);
    addRequirements(drive);
    m_drive = drive;
    holonomicController.setTolerance(new Pose2d(0.02, 0.02, Rotation2d.fromDegrees(1)));
    o_Trajectory = trajectory;
  }

  @Override
  public void initialize() {
    if (isTrajectory) {
      timer.reset();
      timer.start();
      m_Trajectory = o_Trajectory.transformBy(m_drive.getPose().minus(Pose2d.kZero));
    }
  }

  @Override
  public void execute() {
    double time = timer.get();

    ChassisSpeeds chassisSpeeds =
        isTrajectory
            ? holonomicController.calculate(
                m_drive.getPose(), m_Trajectory.sample(time), Rotation2d.kZero)
            : holonomicController.calculate(
                m_drive.getPose(), m_targetPose.get(), 0, Rotation2d.kZero);
    if (holonomicController.atReference()) {
      if (isTrajectory && time < m_Trajectory.getTotalTimeSeconds()) {
        return;
      }
      m_drive.stop();
    } else {
      m_drive.runVelocity(chassisSpeeds);
    }
  }

  @Override
  public void end(boolean interrupted) {
    timer.stop();
  }
}
