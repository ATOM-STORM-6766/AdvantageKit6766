package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/**
 * 通用射击解算器 (Generic Shooter Resolver) v3.0
 *
 * <p>改进点： 1. 使用 Translation3d 描述炮台偏移，统一了水平偏移和高度信息。 2. 移除了 redundant 的 releaseHeightMeters。
 */
public class GenericShooterResolver {

  /** 射击参数配置类 */
  public static class ShooterConfig {
    // --- 物理参数 ---
    /** 射速查找表 (Distance -> Launch Speed)。 Key: 距离 (meters), Value: 发射速度 (m/s) */
    public InterpolatingDoubleTreeMap speedLookup = new InterpolatingDoubleTreeMap();

    /** 重力加速度 (通常为 -9.81) */
    public double gravity = -9.81;
    /** 空气升力系数 */
    public double liftCoefficient = 0.0;

    // --- 机械参数 ---
    /**
     * 炮台发射点相对于机器人中心的 3D 偏移。 X: 前后 (前正) Y: 左右 (左正) Z: 高度 (离地高度为 0 的基准上的高度，通常 Z 就是发射口离地高度)
     *
     * <p>这种表示方式合二为一，既包含了平面位置修正，也包含了高度信息。
     */
    public Translation3d robotCenterToTurret = new Translation3d();

    /** Hood 在 0 度位置时的物理仰角 (弧度)。 */
    public double hoodZeroAngleRadians;
    /** 炮台安装偏差修正 (弧度)。 */
    public double turretCorrectionRadians = 0.0;

    /** Turret yaw limits (radians, robot-relative). */
    public double turretMinYawRadians = Double.NEGATIVE_INFINITY;

    public double turretMaxYawRadians = Double.POSITIVE_INFINITY;

    // --- 限制参数 ---
    public double minRange = 0.0;
    public double maxRange = 999.0;

    public ShooterConfig() {}

    public ShooterConfig withFixedSpeed(double speed) {
      speedLookup.clear();
      speedLookup.put(0.0, speed);
      speedLookup.put(999.0, speed);
      return this;
    }

    public ShooterConfig withSpeedMap(double[] distances, double[] speeds) {
      speedLookup.clear();
      for (int i = 0; i < distances.length; i++) {
        speedLookup.put(distances[i], speeds[i]);
      }
      return this;
    }

    public ShooterConfig withTurretYawLimits(double minYawRadians, double maxYawRadians) {
      this.turretMinYawRadians = minYawRadians;
      this.turretMaxYawRadians = maxYawRadians;
      return this;
    }
  }

  public static class ShooterSetpoint {
    public double turretYaw;
    public double turretFeedforward;
    public double hoodPitch;
    public double hoodFeedforward;
    public double launchSpeed;
    public boolean isValid;

    // Debug fields for visualization
    public Translation3d virtualTarget;
    public Translation3d actualTarget;
    public double flightTime;

    public static ShooterSetpoint invalid() {
      ShooterSetpoint s = new ShooterSetpoint();
      s.isValid = false;
      return s;
    }
  }

  public static ShooterSetpoint resolve(
      Pose2d robotPose,
      ChassisSpeeds robotSpeeds,
      Translation3d targetPosition,
      ShooterConfig config) {

    ShooterSetpoint result = new ShooterSetpoint();

    // --- 步骤 0: 处理炮台 3D 偏移 ---

    // 1. 提取水平偏移 (用于计算场地平面位置)
    Translation2d turretOffsetXY =
        new Translation2d(config.robotCenterToTurret.getX(), config.robotCenterToTurret.getY());

    // 2. Turret 当前的场地位姿 (Translation Only)
    // TurretPos_XY = RobotPos_XY + Rotate(Offset_XY)
    Translation2d turretFieldPosXY =
        robotPose.getTranslation().plus(turretOffsetXY.rotateBy(robotPose.getRotation()));

    // 3. Turret 的绝对高度 (假设 RobotPose 的 Z 是 0，即地面)
    // 如果你的 RobotPose 是 Pose3d 且 Z 不为 0，这里需要加上 robotPose.getZ()
    double turretHeight = config.robotCenterToTurret.getZ();

    // 4. Turret 当前的场地速度 (Field Relative Speed)
    // V_turret = V_robot + Omega x R (只影响水平分量)
    Translation2d tangentialVel =
        turretOffsetXY
            .rotateBy(robotPose.getRotation().plus(Rotation2d.fromDegrees(90)))
            .times(robotSpeeds.omegaRadiansPerSecond);

    double vTurretX = robotSpeeds.vxMetersPerSecond + tangentialVel.getX();
    double vTurretY = robotSpeeds.vyMetersPerSecond + tangentialVel.getY();

    // --- 步骤 1: 计算位移向量 ---
    // RangeVector = Target - Turret
    Translation3d rangeVector =
        targetPosition.minus(
            new Translation3d(turretFieldPosXY.getX(), turretFieldPosXY.getY(), turretHeight));

    double distanceToTarget = new Translation2d(rangeVector.getX(), rangeVector.getY()).getNorm();

    if (distanceToTarget < config.minRange || distanceToTarget > config.maxRange) {
      return ShooterSetpoint.invalid();
    }

    // --- 步骤 2: 获取当前距离下的发射速度 ---
    double vShot = config.speedLookup.get(distanceToTarget);

    // --- 步骤 3: 运动补偿解算 (核心方程) ---
    // |V_shot * t|^2 = |Delta - V_turret * t|^2
    double a = vTurretX * vTurretX + vTurretY * vTurretY - vShot * vShot;

    // Handle edge case: a ≈ 0 (robot velocity close to shot velocity)
    if (Math.abs(a) < 1e-6) {
      vShot = 1.01 * vShot;
      a = vTurretX * vTurretX + vTurretY * vTurretY - vShot * vShot;
    }

    double b = -2.0 * (rangeVector.getX() * vTurretX + rangeVector.getY() * vTurretY);
    double c =
        rangeVector.getX() * rangeVector.getX()
            + rangeVector.getY() * rangeVector.getY()
            + rangeVector.getZ() * rangeVector.getZ();

    double discriminant = b * b - 4.0 * a * c;

    // Handle edge case: discriminant < 0 (no geometric solution, use approximate)
    if (discriminant < 0.0) {
      discriminant = 0.0;
    }

    double t = (-b - Math.sqrt(discriminant)) / (2.0 * a);
    if (t <= 0) return ShooterSetpoint.invalid();

    // --- 步骤 4: 计算虚拟目标和 Turret 角度 ---
    Translation3d virtualShotVelocity =
        new Translation3d(
            (rangeVector.getX() - vTurretX * t) / t,
            (rangeVector.getY() - vTurretY * t) / t,
            (rangeVector.getZ() / t));

    Rotation2d fieldRelativeTurretYaw =
        new Rotation2d(virtualShotVelocity.getX(), virtualShotVelocity.getY());

    // 转换为 Robot Relative
    Rotation2d robotRelativeTurretYaw = fieldRelativeTurretYaw.minus(robotPose.getRotation());
    result.turretYaw = robotRelativeTurretYaw.getRadians() + config.turretCorrectionRadians;
    if (result.turretYaw < config.turretMinYawRadians
        || result.turretYaw > config.turretMaxYawRadians) {
      return ShooterSetpoint.invalid();
    }

    // --- 步骤 5: 计算 Hood 角度 ---
    double xyVel = Math.hypot(virtualShotVelocity.getX(), virtualShotVelocity.getY());
    double drop = 0.5 * config.gravity * t * t;
    if (config.liftCoefficient != 0) {
      drop += 0.5 * config.liftCoefficient * c;
    }

    double pitchAngleRads = Math.atan2((rangeVector.getZ() - drop) / t, xyVel);
    result.hoodPitch = config.hoodZeroAngleRadians - pitchAngleRads;

    // --- 步骤 6: 计算前馈 ---
    Translation2d turretToTargetXY = new Translation2d(rangeVector.getX(), rangeVector.getY());
    Translation2d velocityInTargetFrame =
        new Translation2d(vTurretX, vTurretY).rotateBy(turretToTargetXY.getAngle().unaryMinus());

    double tangentialComponent = velocityInTargetFrame.getY();
    double radialComponent = velocityInTargetFrame.getX();

    result.turretFeedforward =
        (tangentialComponent / distanceToTarget) - robotSpeeds.omegaRadiansPerSecond;

    double h = rangeVector.getZ();
    result.hoodFeedforward = radialComponent * h / (distanceToTarget * distanceToTarget + h * h);

    // --- 步骤 7: 最终速度 ---
    double vzCorrected = (rangeVector.getZ() - drop) / t;
    result.launchSpeed = Math.hypot(xyVel, vzCorrected);
    result.isValid = true;

    // --- 步骤 8: 填充调试字段 ---
    result.flightTime = t;
    result.actualTarget = targetPosition;

    // Virtual target: where we're actually aiming (compensated for motion)
    // This is the turret position plus the virtual shot velocity direction scaled by distance
    Translation3d turretPos3d =
        new Translation3d(turretFieldPosXY.getX(), turretFieldPosXY.getY(), turretHeight);
    double virtualDistance = virtualShotVelocity.getNorm() * t;
    Translation3d virtualDirection =
        new Translation3d(
            virtualShotVelocity.getX() / virtualShotVelocity.getNorm(),
            virtualShotVelocity.getY() / virtualShotVelocity.getNorm(),
            vzCorrected / result.launchSpeed);
    result.virtualTarget = turretPos3d.plus(virtualDirection.times(virtualDistance));

    return result;
  }
}
