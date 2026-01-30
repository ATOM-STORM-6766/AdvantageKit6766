package frc.robot.commands;

import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;

public class FollowChoreoPathCommand extends Command {
  private static PIDController trans = new PIDController(3, 0, 0); // TODO 移动到constants
  private static ProfiledPIDController rotation =
      new ProfiledPIDController(
          5, 0, 0, new TrapezoidProfile.Constraints(6.28, 3.14)); // TODO 移动到constants
  private static HolonomicDriveController holonomicController =
      new HolonomicDriveController(trans, trans, rotation);

  static {
    rotation.enableContinuousInput(-Math.PI, Math.PI);
    holonomicController.setTolerance(new Pose2d(0.02, 0.02, Rotation2d.fromDegrees(1)));
  }

  private Drive m_drive;

  private Timer timer = new Timer();
  private Trajectory<SwerveSample> m_Trajectory;

  public FollowChoreoPathCommand(Drive drive, Trajectory<SwerveSample> trajectory) {
    addRequirements(drive);
    m_drive = drive;
    m_Trajectory = trajectory;
  }

  @Override
  public void initialize() {
    timer.reset();
    timer.start();
  }

  @Override
  public void execute() {
    double time = timer.get();
    var sample = m_Trajectory.sampleAt(time, false).get();
    ChassisSpeeds chassisSpeeds =
        holonomicController.calculate(
            m_drive.getPose(),
            sample.getPose(),
            Math.hypot(sample.vx, sample.vy),
            Rotation2d.fromRadians(sample.heading));
    m_drive.runVelocity(chassisSpeeds);
  }

  @Override
  public boolean isFinished() {
    return holonomicController.atReference() && timer.get() >= m_Trajectory.getTotalTime();
  }

  @Override
  public void end(boolean interrupted) {
    timer.stop();
  }
}
