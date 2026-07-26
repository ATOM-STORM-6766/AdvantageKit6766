// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// 本程序是自由软件；您可以按照自由软件基金会发布的第三版 GNU 通用公共许可证
// 的条款，重新分发和/或修改本程序，许可证副本可在本项目根目录找到。
//
// 本程序的发布目的是希望它有用，但不提供任何保证；甚至没有适销性或针对某一
// 特定用途适用性的默示保证。详情参见 GNU 通用公共许可证。

package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.FieldConstants;

public class VisionConstants {
  // AprilTag 布局
  public static AprilTagFieldLayout aprilTagLayout = FieldConstants.getAprilTagLayout();

  // 摄像头名称，必须与协处理器上配置的名称一致
  public static String camera0Name = "Back_Camera";
  public static String camera1Name = "Left_Camera";
  public static String camera2Name = "Right_Camera";

  // 机器人到摄像头的变换
  // （Limelight 不使用，在其 Web UI 中配置）
  public static Transform3d robotToCamera0 = // 坐标已经更新前为0 后为1
      new Transform3d(-0.31784, 0.0, 0.30452, new Rotation3d(0.0, Math.toRadians(30), Math.PI));

  public static Transform3d robotToCamera1 =
      new Transform3d(
          -0.17500, 0.334924, 0.340868, new Rotation3d(0.0, Math.toRadians(10.0), Math.PI / 2));

  public static Transform3d robotToCamera2 =
      new Transform3d(
          -0.007500, -0.334924, 0.340868, new Rotation3d(0.0, Math.toRadians(-10.0), -Math.PI / 2));

  // 基础过滤阈值
  public static double maxAmbiguity = 0.3;
  public static double maxZError = 0.75;

  // 标准差基线，基于 1 米距离且只有 1 个标签
  // （会根据距离与标签数量自动调整）
  public static double linearStdDevBaseline = 0.02 * 3.0; // 单位：米
  public static double angularStdDevBaseline = 0.06 * 3.0; // 单位：弧度

  // 每个摄像头的标准差乘数
  // （可根据需求调整以提高某些摄像头的权重）
  public static double[] cameraStdDevFactors =
      new double[] {
        0.25, // 摄像头 0
        1.0, // 摄像头 1
        1.0 // 摄像头 2
      };

  // MegaTag 2 观测的乘数
  public static double linearStdDevMegatag2Factor = 0.5; // 比完整 3D 解更稳定
  public static double angularStdDevMegatag2Factor = Double.POSITIVE_INFINITY; // 无可用的旋转数据
}
