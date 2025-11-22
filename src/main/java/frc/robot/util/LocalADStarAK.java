package frc.robot.util;

import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPoint;
import com.pathplanner.lib.pathfinding.LocalADStar;
import com.pathplanner.lib.pathfinding.Pathfinder;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;

// 说明：此文件可在以下链接获取
// https://gist.github.com/mjansen4857/a8024b55eb427184dbd10ae8923bd57d

public class LocalADStarAK implements Pathfinder {
  private final ADStarIO io = new ADStarIO();

  /**
   * 判断自上次获取路径以来，是否已计算出新的路径。
   *
   * @return 若有可用的新路径则返回 true
   */
  @Override
  public boolean isNewPathAvailable() {
    if (!Logger.hasReplaySource()) {
      io.updateIsNewPathAvailable();
    }

    Logger.processInputs("LocalADStarAK", io);

    return io.isNewPathAvailable;
  }

  /**
   * 获取最近一次计算出的路径。
   *
   * @param constraints 构建路径时要使用的路径约束
   * @param goalEndState 构建路径时的目标结束状态
   * @return 根据寻路器计算出的点创建的 PathPlannerPath
   */
  @Override
  public PathPlannerPath getCurrentPath(PathConstraints constraints, GoalEndState goalEndState) {
    if (!Logger.hasReplaySource()) {
      io.updateCurrentPathPoints(constraints, goalEndState);
    }

    Logger.processInputs("LocalADStarAK", io);

    if (io.currentPathPoints.isEmpty()) {
      return null;
    }

    return PathPlannerPath.fromPathPoints(io.currentPathPoints, constraints, goalEndState);
  }

  /**
   * 设置寻路的起始位置。
   *
   * @param startPosition 场地上的起点；若位于障碍物内，将被移动到最近的非障碍节点。
   */
  @Override
  public void setStartPosition(Translation2d startPosition) {
    if (!Logger.hasReplaySource()) {
      io.adStar.setStartPosition(startPosition);
    }
  }

  /**
   * 设置寻路的目标位置。
   *
   * @param goalPosition 场地上的目标点；若位于障碍物内，将被移动到最近的非障碍节点。
   */
  @Override
  public void setGoalPosition(Translation2d goalPosition) {
    if (!Logger.hasReplaySource()) {
      io.adStar.setGoalPosition(goalPosition);
    }
  }

  /**
   * 设置寻路过程中需要避开的动态障碍。
   *
   * @param obs 表示障碍物的 Translation2d 成对列表，每对点代表包围盒的对角。
   * @param currentRobotPos 机器人当前位置，用于根据障碍调整路径初始位置。
   */
  @Override
  public void setDynamicObstacles(
      List<Pair<Translation2d, Translation2d>> obs, Translation2d currentRobotPos) {
    if (!Logger.hasReplaySource()) {
      io.adStar.setDynamicObstacles(obs, currentRobotPos);
    }
  }

  private static class ADStarIO implements LoggableInputs {
    public LocalADStar adStar = new LocalADStar();
    public boolean isNewPathAvailable = false;
    public List<PathPoint> currentPathPoints = Collections.emptyList();

    @Override
    public void toLog(LogTable table) {
      table.put("IsNewPathAvailable", isNewPathAvailable);

      double[] pointsLogged = new double[currentPathPoints.size() * 2];
      int idx = 0;
      for (PathPoint point : currentPathPoints) {
        pointsLogged[idx] = point.position.getX();
        pointsLogged[idx + 1] = point.position.getY();
        idx += 2;
      }

      table.put("CurrentPathPoints", pointsLogged);
    }

    @Override
    public void fromLog(LogTable table) {
      isNewPathAvailable = table.get("IsNewPathAvailable", false);

      double[] pointsLogged = table.get("CurrentPathPoints", new double[0]);

      List<PathPoint> pathPoints = new ArrayList<>();
      for (int i = 0; i < pointsLogged.length; i += 2) {
        pathPoints.add(
            new PathPoint(new Translation2d(pointsLogged[i], pointsLogged[i + 1]), null));
      }

      currentPathPoints = pathPoints;
    }

    public void updateIsNewPathAvailable() {
      isNewPathAvailable = adStar.isNewPathAvailable();
    }

    public void updateCurrentPathPoints(PathConstraints constraints, GoalEndState goalEndState) {
      PathPlannerPath currentPath = adStar.getCurrentPath(constraints, goalEndState);

      if (currentPath != null) {
        currentPathPoints = currentPath.getAllPathPoints();
      } else {
        currentPathPoints = Collections.emptyList();
      }
    }
  }
}
