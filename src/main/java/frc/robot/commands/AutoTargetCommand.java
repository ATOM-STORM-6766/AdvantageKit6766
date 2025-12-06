package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.Supplier;

/** 自动寻找并前往最近目标点的命令 会从预设的目标点列表中找出最优的目标点，并自动规划路径前往该点 评分标准同时考虑位置距离和方位差异 */
public class AutoTargetCommand extends Command {
  private final Supplier<Pose2d> m_targetPose;
  private Command m_pathfindCommand = null;

  // 路径约束配置
  private static final PathConstraints constraints =
      new PathConstraints(
          70.300, // 最大速度 (m/s)
          100.900, // 最大加速度 (m/s²)
          2820.996 * Math.PI / 180, // 最大角速度 (deg/s)
          2000 * Math.PI / 180 // 最大角加速度 (deg/s²)
          );

  /**
   * 构造函数
   *
   * @param swerve 底盘子系统
   * @param targetPose 目标位姿列表
   */
  public AutoTargetCommand(Supplier<Pose2d> targetPose) {
    m_targetPose = targetPose;
  }

  @Override
  public void initialize() {
    // 创建寻路命令
    m_pathfindCommand = AutoBuilder.pathfindToPose(m_targetPose.get(), constraints, 0.0); // 目标点速度为0

    // 开始执行寻路命令
    m_pathfindCommand.initialize();
  }

  @Override
  public void execute() {
    if (m_pathfindCommand != null) {
      m_pathfindCommand.execute();
    }
  }

  @Override
  public void end(boolean interrupted) {
    if (m_pathfindCommand != null) {
      m_pathfindCommand.end(interrupted);
    }
  }

  @Override
  public boolean isFinished() {
    return m_pathfindCommand != null && m_pathfindCommand.isFinished();
  }
}
