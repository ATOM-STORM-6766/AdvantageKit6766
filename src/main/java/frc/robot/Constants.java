// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import com.ctre.phoenix6.CANBus;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.generated.TunerConstants;
import java.util.List;
import frc.robot.subsystems.turret.TurretConstants;
import frc.robot.util.GenericShooterResolver.ShooterConfig;
import frc.robot.util.GenericShooterResolverV2.ShooterConfigV2;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static final CANBus kCANBus = new CANBus("rio");

  // Turret offset from robot center (X: forward, Y: left, Z: height)
  public static final Translation3d ROBOT_CENTER_TO_TURRET = new Translation3d(-0.19685, 0.0, 0.44);

  public static final ShooterConfig SHOOTER_CONFIG = createShooterConfig();
  public static final ShooterConfigV2 SHOOTER_CONFIG_V2 = createShooterConfigV2();

  private static ShooterConfig createShooterConfig() {
    ShooterConfig config = new ShooterConfig();

    // Hood 0 degrees = vertical (pointing up), so hoodZeroAngleRadians = π/2
    config.hoodZeroAngleRadians = Math.PI / 2.0;

    // Turret offset from robot center
    config.robotCenterToTurret = ROBOT_CENTER_TO_TURRET;

    // Turret yaw limits
    config.turretMinYawRadians = TurretConstants.kTurretMinPositionRadians;
    config.turretMaxYawRadians = TurretConstants.kTurretMaxPositionRadians;

    // Default fixed launch speed for testing (15 m/s)
    config.withFixedSpeed(15.0);

    return config;
  }

  private static ShooterConfigV2 createShooterConfigV2() {
    ShooterConfigV2 config = new ShooterConfigV2();

    config.turretMaxYawRadians = TurretConstants.kTurretMaxPositionRadians;
    config.turretMinYawRadians = TurretConstants.kTurretMinPositionRadians;

    config.robotCenterToTurret = ROBOT_CENTER_TO_TURRET;

    config.hoodPitchRadiansMap.put(1.45, Rotation2d.fromDegrees(19.0));
    config.hoodPitchRadiansMap.put(1.75, Rotation2d.fromDegrees(21.0));
    config.hoodPitchRadiansMap.put(2.15, Rotation2d.fromDegrees(22.0));
    config.hoodPitchRadiansMap.put(2.50, Rotation2d.fromDegrees(23.0));
    config.hoodPitchRadiansMap.put(2.84, Rotation2d.fromDegrees(24.0));
    config.hoodPitchRadiansMap.put(3.15, Rotation2d.fromDegrees(25.5));
    config.hoodPitchRadiansMap.put(3.58, Rotation2d.fromDegrees(26.5));
    config.hoodPitchRadiansMap.put(4.16, Rotation2d.fromDegrees(29.0));
    config.hoodPitchRadiansMap.put(4.43, Rotation2d.fromDegrees(30.5));
    config.hoodPitchRadiansMap.put(5.28, Rotation2d.fromDegrees(34.0));

    config.flywheelRpsMap.put(1.45, 175.0);
    config.flywheelRpsMap.put(1.75, 185.0);
    config.flywheelRpsMap.put(2.15, 190.0);
    config.flywheelRpsMap.put(2.50, 200.0);
    config.flywheelRpsMap.put(2.84, 210.0);
    config.flywheelRpsMap.put(3.15, 218.0);
    config.flywheelRpsMap.put(3.58, 222.0);
    config.flywheelRpsMap.put(4.16, 230.0);
    config.flywheelRpsMap.put(4.43, 235.0);
    config.flywheelRpsMap.put(5.28, 250.0);

    config.timeOfFlightSecondsMap.put(1.64227, 0.93);
    config.timeOfFlightSecondsMap.put(2.859544, 1.0);
    config.timeOfFlightSecondsMap.put(4.27071, 1.05);

    return config;
  }

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static final Trajectory trajectoryOne =
      TrajectoryGenerator.generateTrajectory(
          new Pose2d(0, 0, Rotation2d.fromDegrees(0)), // 起始位姿：(x=0, y=0)，朝向 0 度
          List.of(
              new Translation2d(0.0, 0.5),
              new Translation2d(0.5, 0.5),
              new Translation2d(0.5, 0.0)), // 中间途经点：(0.5, 0.5) 和 (2, -0.5)
          new Pose2d(0.0, 0, Rotation2d.fromDegrees(0)), // 结束位姿：(x=3, y=0)，朝向 0 度
          new TrajectoryConfig(
              TunerConstants.kSpeedAt12Volts.times(0.3),
              MetersPerSecondPerSecond.of(1.3).times(0.3))); // 配置：最大速度和加速度均为 3 英尺/秒
}
