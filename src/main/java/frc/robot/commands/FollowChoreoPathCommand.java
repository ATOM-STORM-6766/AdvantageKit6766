package frc.robot.commands;

import choreo.trajectory.EventMarker;
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
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.AllianceFlipUtil;
import java.util.Map;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;

public class FollowChoreoPathCommand extends Command {
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

  private Timer timer = new Timer();
  private Trajectory<SwerveSample> trajectory;
  private Trajectory<SwerveSample> m_trajectory;
  private BooleanSupplier m_isRed = AllianceFlipUtil::shouldFlip;
  /**
   * Create an auto routine that handles path splitting and events.
   *
   * @param drive The drive subsystem.
   * @param trajectory The full trajectory.
   * @param eventMap A map of event names to commands.
   * @param splitEvents A map of split indices to commands to run between segments. Key is the index
   *     of the segment that just finished (0-indexed).
   * @return A command (likely SequentialCommandGroup) following the path and events.
   */
  public static Command createAutoRoutine(
      Drive drive,
      Trajectory<SwerveSample> trajectory,
      Map<String, Command> eventMap,
      Map<Integer, Command> splitEvents) {
    // if (trajectory.splits().isEmpty()) {
    // return new FollowChoreoPathCommand(drive, Optional.of(trajectory));
    // }

    SequentialCommandGroup group = new SequentialCommandGroup();
    // Assuming getSplit(i) returns the i-th segment (0 to splits.size())
    for (int i = 0; i <= trajectory.splits().size(); i++) {
      // getSplit may return empty if the index is out of bounds or split is invalid
      // Although the loop condition should be safe, check for presence
      final int index = i;
      Optional<Trajectory<SwerveSample>> splitOptional = trajectory.getSplit(i);

      if (splitOptional.isPresent()) {
        Trajectory<SwerveSample> split = splitOptional.get();
        group.addCommands(new FollowChoreoPathCommand(drive, Optional.of(split)));
        if (splitEvents.containsKey(index)) {
          group.addCommands(splitEvents.get(index));
        }
      }
    }
    ParallelCommandGroup parallelEvents = new ParallelCommandGroup(group);
    for (EventMarker marker : trajectory.events()) {
      System.out.println("Event: " + marker.event + " at " + marker.timestamp);
      if (eventMap.containsKey(marker.event)) {
        parallelEvents.addCommands(
            eventMap
                .get(marker.event)
                .beforeStarting(Commands.waitSeconds(marker.timestamp))
                .withName("Event: " + marker.event));
      }
    }
    parallelEvents.addCommands(
        Commands.print(Integer.valueOf(parallelEvents.getRequirements().size()).toString()));
    return parallelEvents;
  }

  /**
   * Create an auto routine that handles path splitting and events.
   *
   * @param drive The drive subsystem.
   * @param trajectory The full trajectory.
   * @param eventMap A map of event names to commands.
   * @return A command (likely SequentialCommandGroup) following the path and events.
   */
  public static Command createAutoRoutine(
      Drive drive, Trajectory<SwerveSample> trajectory, Map<String, Command> eventMap) {
    return createAutoRoutine(drive, trajectory, eventMap, Map.of());
  }

  /**
   * 构造一个新的 FollowChoreoPathCommand with events.
   *
   * @param drive 机器人驱动子系统 (Drive)，该命令需要独占使用它。
   * @param trajectory 要跟随的 Choreo 路径轨迹 以蓝方为参考。
   */
  public FollowChoreoPathCommand(Drive drive, Optional<Trajectory<SwerveSample>> trajectory) {
    addRequirements(drive);
    m_drive = drive;
    this.trajectory = trajectory.get();
  }

  @Override
  public void initialize() {
    m_trajectory = m_isRed.getAsBoolean() ? trajectory.flipped() : trajectory;
    timer.reset();
    timer.start();
    m_drive.setPose(m_trajectory.getInitialPose(false).get());
    Logger.recordOutput("Odometry/Trajectory", m_trajectory.getPoses());
    Logger.recordOutput("Odometry/TrajectorySetpoint", m_trajectory.getInitialPose(false).get());
  }

  @Override
  public void execute() {
    double time = timer.get();

    var sample = m_trajectory.sampleAt(time, false).get();
    Logger.recordOutput("Odometry/TrajectorySetpoint", sample.getPose());
    ChassisSpeeds chassisSpeeds =
        ChassisSpeeds.fromFieldRelativeSpeeds( // TODO 测试是否添加延迟补偿的效果
                sample.vx, sample.vy, sample.omega, sample.getPose().getRotation())
            .plus(
                holonomicController.calculate(
                    m_drive.getPose(),
                    sample.getPose(),
                    0,
                    Rotation2d.fromRadians(sample.heading)));
    m_drive.runVelocity(chassisSpeeds);
  }

  @Override
  public boolean isFinished() {
    return holonomicController.atReference() && timer.get() >= m_trajectory.getTotalTime();
  }

  @Override
  public void end(boolean interrupted) {
    timer.stop();
  }
}
