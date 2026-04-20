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
    return config;
  }

  private static ShooterConfig createShooterConfig() {
    ShooterConfig config = new ShooterConfig();

    config.robotCenterToTurret = new Translation3d(-0.235, 0.0, 0.63);

    config.minHoodPitchDegrees = HoodConstants.kHoodMinPosition.in(Degree);
    config.maxHoodPitchDegrees = HoodConstants.kHoodMaxPosition.in(Degree);

    config.maxFlywheelRps = 100 / 1.2;

    config.hoodPitchRadiansMap.put(1.50, Rotation2d.fromDegrees(25.0));
    config.hoodPitchRadiansMap.put(1.90, Rotation2d.fromDegrees(25.5));
    config.hoodPitchRadiansMap.put(2.30, Rotation2d.fromDegrees(28.0));
    config.hoodPitchRadiansMap.put(2.70, Rotation2d.fromDegrees(30.0));
    config.hoodPitchRadiansMap.put(3.10, Rotation2d.fromDegrees(30.0));
    config.hoodPitchRadiansMap.put(3.50, Rotation2d.fromDegrees(30.0));
    config.hoodPitchRadiansMap.put(3.90, Rotation2d.fromDegrees(34.0));
    config.hoodPitchRadiansMap.put(4.30, Rotation2d.fromDegrees(35.0));
    config.hoodPitchRadiansMap.put(4.70, Rotation2d.fromDegrees(35.0));
    config.hoodPitchRadiansMap.put(5.10, Rotation2d.fromDegrees(35.0));

    config.flywheelRpsMap.put(1.50, 43.0 + 2);
    config.flywheelRpsMap.put(1.90, 47.0 + 2);
    config.flywheelRpsMap.put(2.30, 50.5 + 2);
    config.flywheelRpsMap.put(2.70, 51.0 + 2);
    config.flywheelRpsMap.put(3.10, 55.5 + 2);
    config.flywheelRpsMap.put(3.50, 55.5 + 2);
    config.flywheelRpsMap.put(3.90, 55.0 + 2);
    config.flywheelRpsMap.put(4.30, 62.0 + 2);
    config.flywheelRpsMap.put(4.70, 61.5 + 2);
    config.flywheelRpsMap.put(5.10, 64.0 + 2);

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
    config.maxRange = 5.1;

    return config;
  }

  public static class DriveControlConstants {
    public static class TrajectoryFollow {
      public static final double TRANS_X_KP = 0.08;
      public static final double TRANS_X_KI = 0.0;
      public static final double TRANS_X_KD = 0.001;

      public static final double TRANS_Y_KP = 0.06;
      public static final double TRANS_Y_KI = 0.0;
      public static final double TRANS_Y_KD = 0.001;

      public static final double ROTATION_KP = 0.015;
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

      private FollowPoint() {}
    }

    public static class JoystickAngleHold {
      public static final double KP = 2.0;
      public static final double KI = 0.0;
      public static final double KD = 0.1;
      public static final double MAX_VELOCITY = 13.200;
      public static final double MAX_ACCELERATION = 36.366 / 2.0;
      public static final double TOLERANCE_RAD = Units.degreesToRadians(2.0);

      public static final double FF_KS = 0.0;
      public static final double FF_KV = 0.1;
      public static final double FF_KA = 0.2;

      public static ProfiledPIDController createAngleController() {
        ProfiledPIDController controller =
            new ProfiledPIDController(
                KP, KI, KD, new TrapezoidProfile.Constraints(MAX_VELOCITY, MAX_ACCELERATION));
        controller.enableContinuousInput(-Math.PI, Math.PI);
        controller.setTolerance(TOLERANCE_RAD);
        return controller;
      }

      public static SimpleMotorFeedforward createAngleFeedforward() {
        return new SimpleMotorFeedforward(FF_KS, FF_KV, FF_KA);
      }

      private JoystickAngleHold() {}
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
