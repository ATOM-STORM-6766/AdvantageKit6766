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

import static edu.wpi.first.units.Units.Degree;

import com.ctre.phoenix6.CANBus;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.hood.HoodConstants;
import frc.robot.util.GenericShooterResolver.ShooterConfig;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static final CANBus kCANBus = CANBus.roboRIO();

  public static final ShooterConfig SHOOTER_CONFIG = createShooterConfig();
  public static final ShooterConfig PASS_CONFIG = createPassConfig();

  private static ShooterConfig createPassConfig() {
    ShooterConfig config = createShooterConfig();
    config.restrictToAllianceForward = false;
    config.hoodPitchRadiansMap.clear();
    config.flywheelRpsMap.clear();

    config.hoodPitchRadiansMap.put(1.50, Rotation2d.fromDegrees(44.0));

    config.flywheelRpsMap.put(1.50, 35.0);

    // TODO 必须与实际的插值表范围相匹配
    config.minRange = 1.5;
    config.maxRange = 4.0;
    return config;
  }

  private static ShooterConfig createShooterConfig() {
    ShooterConfig config = new ShooterConfig();

    config.robotCenterToTurret = new Translation3d(-0.178, 0.0, 0.50);

    config.shootingDirection = Rotation2d.kPi;

    config.minHoodPitchDegrees = HoodConstants.kHoodMinPosition.in(Degree);
    config.maxHoodPitchDegrees = HoodConstants.kHoodMaxPosition.in(Degree);

    config.maxFlywheelRps = 100 / 1.2;

    config.hoodPitchRadiansMap.put(1.50, Rotation2d.fromDegrees(15.0));
    config.hoodPitchRadiansMap.put(1.80, Rotation2d.fromDegrees(15.0));
    config.hoodPitchRadiansMap.put(2.10, Rotation2d.fromDegrees(16.0));
    config.hoodPitchRadiansMap.put(2.40, Rotation2d.fromDegrees(18.0));
    config.hoodPitchRadiansMap.put(2.70, Rotation2d.fromDegrees(20.0));
    config.hoodPitchRadiansMap.put(3.00, Rotation2d.fromDegrees(22.0));
    config.hoodPitchRadiansMap.put(3.50, Rotation2d.fromDegrees(25.0));
    config.hoodPitchRadiansMap.put(4.00, Rotation2d.fromDegrees(31.0));

    config.flywheelRpsMap.put(1.50, 26.8 + 0.5);
    config.flywheelRpsMap.put(1.80, 28.0 + 0.5);
    config.flywheelRpsMap.put(2.10, 30.0 + 0.5);
    config.flywheelRpsMap.put(2.40, 30.0 + 0.5);
    config.flywheelRpsMap.put(2.70, 31.0 + 0.5);
    config.flywheelRpsMap.put(3.00, 32.0 + 0.5);
    config.flywheelRpsMap.put(3.20, 33.3 + 1.5);
    config.flywheelRpsMap.put(3.50, 34.0 + 0.5);
    config.flywheelRpsMap.put(4.00, 34.0 + 0.5);

    config.timeOfFlightSecondsMap.put(1.50, 0.95);
    config.timeOfFlightSecondsMap.put(1.90, 0.98);
    config.timeOfFlightSecondsMap.put(2.30, 1.10);
    config.timeOfFlightSecondsMap.put(2.70, 1.11);
    config.timeOfFlightSecondsMap.put(3.10, 1.26);
    config.timeOfFlightSecondsMap.put(3.50, 1.28);
    config.timeOfFlightSecondsMap.put(3.90, 1.29);
    config.timeOfFlightSecondsMap.put(4.30, 1.35);
    config.timeOfFlightSecondsMap.put(4.70, 1.35);
    config.timeOfFlightSecondsMap.put(5.10, 1.36);

    // TODO 必须与实际的插值表范围相匹配
    config.minRange = 1.5;
    config.maxRange = 4.0;

    return config;
  }

  public static class DriveControlConstants {
    public static class TrajectoryFollow {
      public static final double TRANS_X_KP = 0.8;
      public static final double TRANS_X_KI = 0.0015;
      public static final double TRANS_X_KD = 0.000;

      public static final double TRANS_Y_KP = 0.8;
      public static final double TRANS_Y_KI = 0.0015;
      public static final double TRANS_Y_KD = 0.000;

      public static final double ROTATION_KP = 0.5;
      public static final double ROTATION_KI = 0.0;
      public static final double ROTATION_KD = 0.005;

      public static final double ROTATION_TOLERANCE_RAD = Units.degreesToRadians(2.0);

      public static final PIDController transXController =
          new PIDController(TRANS_X_KP, TRANS_X_KI, TRANS_X_KD);
      public static final PIDController transYController =
          new PIDController(TRANS_Y_KP, TRANS_Y_KI, TRANS_Y_KD);
      public static final PIDController rotationController =
          new PIDController(ROTATION_KP, ROTATION_KI, ROTATION_KD);

      static {
        rotationController.enableContinuousInput(-Math.PI, Math.PI);
        rotationController.setTolerance(ROTATION_TOLERANCE_RAD);
      }

      public static void publishToSmartDashboard() {
        SmartDashboard.putData("TrajectoryFollow/TransX", transXController);
        SmartDashboard.putData("TrajectoryFollow/TransY", transYController);
        SmartDashboard.putData("TrajectoryFollow/Rotation", rotationController);
      }

      private TrajectoryFollow() {}
    }

    public static class FollowPoint {
      public static final double TRANSLATION_KP = 5.0;
      public static final double TRANSLATION_KI = 0.0;
      public static final double TRANSLATION_KD = 0.0;

      public static final double ROTATION_KP = 5.0;
      public static final double ROTATION_KI = 0.0;
      public static final double ROTATION_KD = 0.0;
      public static final double ROTATION_MAX_VELOCITY = 13.200;
      public static final double ROTATION_MAX_ACCELERATION = 36.366;

      public static final Pose2d CONTROLLER_TOLERANCE =
          new Pose2d(0.02, 0.02, Rotation2d.fromDegrees(1.0));

      public static final PIDController translationController =
          new PIDController(TRANSLATION_KP, TRANSLATION_KI, TRANSLATION_KD);
      public static final ProfiledPIDController rotationController =
          new ProfiledPIDController(
              ROTATION_KP,
              ROTATION_KI,
              ROTATION_KD,
              new TrapezoidProfile.Constraints(ROTATION_MAX_VELOCITY, ROTATION_MAX_ACCELERATION));
      public static final HolonomicDriveController holonomicController =
          new HolonomicDriveController(
              translationController, translationController, rotationController);

      static {
        holonomicController.setTolerance(CONTROLLER_TOLERANCE);
      }

      public static void publishToSmartDashboard() {
        SmartDashboard.putData("FollowPoint/Translation", translationController);
        SmartDashboard.putData("FollowPoint/Rotation", rotationController);
      }

      private FollowPoint() {}
    }

    public static class JoystickAngleHold {
      public static final double KP = 2.5;
      public static final double KI = 0.0;
      public static final double KD = 0.1;
      public static final double MAX_VELOCITY = 13.200;
      public static final double MAX_ACCELERATION = 36.366 * 1.5; // rad/s²，原 36.366，增加以改善陀螺噪声抖动
      public static final double TOLERANCE_RAD = Units.degreesToRadians(2.0);

      public static final double FF_KS = 0.0;
      public static final double FF_KV = 0.1;
      public static final double FF_KA = 0.2;

      public static final ProfiledPIDController angleController =
          new ProfiledPIDController(
              KP, KI, KD, new TrapezoidProfile.Constraints(MAX_VELOCITY, MAX_ACCELERATION));
      public static final SimpleMotorFeedforward angleFeedforward =
          new SimpleMotorFeedforward(FF_KS, FF_KV, FF_KA);

      static {
        angleController.enableContinuousInput(-Math.PI, Math.PI);
        angleController.setTolerance(TOLERANCE_RAD);
      }

      public static void publishToSmartDashboard() {
        SmartDashboard.putData("JoystickAngleHold/Angle", angleController);
      }

      private JoystickAngleHold() {}
    }

    public static class FollowPointConstants {
      public static final double DRIVE_KP = 3.5;
      public static final double DRIVE_KD = 0.0;
      public static final double THETA_KP = 3.5;
      public static final double THETA_KD = 0.1;

      public static final double DRIVE_MAX_VELOCITY = 6.0;
      public static final double DRIVE_MAX_ACCELERATION = 9.0;
      public static final double THETA_MAX_VELOCITY = 11.2;
      public static final double THETA_MAX_ACCELERATION = 14.0;

      public static final double DRIVE_TOLERANCE = 0.10;
      public static final double THETA_TOLERANCE = Units.degreesToRadians(2.0);
      public static final double SETPOINT_MIN_VELOCITY = -0.5;
      public static final double MIN_DISTANCE_VELOCITY_CORRECTION = 0.01;
      public static final double LOOKAHEAD_SECS = 0.30;

      private FollowPointConstants() {}
    }

    public static class MoveToTrenchConstants {
      // 平移：参考 TrajectoryFollow(0.8) 与 FollowPoint(5.0) 之间，带前馈结构可适当偏高
      public static final double DRIVE_KP = 3.5;
      public static final double DRIVE_KD = 0.15; // 微分阻尼，抑制长距离过冲

      // 旋转：KP 参考 JoystickAngleHold(2.0) 与 FollowPoint(5.0) 取中间值
      // KD 对齐 JoystickAngleHold(0.1)，原值 0.5 会放大陀螺噪声
      public static final double THETA_KP = 3.0;
      public static final double THETA_KD = 0.1;

      // 平移限速：最大速度保守，加速度降低以改善启动平滑性
      public static final double DRIVE_MAX_VELOCITY = 5.0;
      public static final double DRIVE_MAX_ACCELERATION = 8.0;

      // 旋转限速：对齐 JoystickAngleHold 已验证的实测值
      public static final double THETA_MAX_VELOCITY = 11.2; // rad/s，原 8.73
      public static final double THETA_MAX_ACCELERATION = 14.0; // rad/s²，原 8.0

      public static final double DRIVE_TOLERANCE = 0.10;
      public static final double THETA_TOLERANCE = Units.degreesToRadians(2.0);
      public static final double SETPOINT_MIN_VELOCITY = -0.5;
      public static final double MIN_DISTANCE_VELOCITY_CORRECTION = 0.01;

      private MoveToTrenchConstants() {}
    }

    private DriveControlConstants() {}
  }

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }
}
