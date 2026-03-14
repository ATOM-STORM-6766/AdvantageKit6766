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
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
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

  public static final CANBus kCANBus = new CANBus("rio");

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

    config.flywheelRpsMap.put(1.50, 43.0 + 2.5);
    config.flywheelRpsMap.put(1.90, 47.0 + 2.5);
    config.flywheelRpsMap.put(2.30, 50.5 + 2.5);
    config.flywheelRpsMap.put(2.70, 51.0 + 2.5);
    config.flywheelRpsMap.put(3.10, 55.5 + 2.5);
    config.flywheelRpsMap.put(3.50, 55.5 + 2.5);
    config.flywheelRpsMap.put(3.90, 55.0 + 2.5);
    config.flywheelRpsMap.put(4.30, 62.0 + 2.5);
    config.flywheelRpsMap.put(4.70, 61.5 + 2.5);
    config.flywheelRpsMap.put(5.10, 64.0 + 2.5);

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

  public static class AutoConstants {
    public static PIDController transX = new PIDController(0.08, 0, 0.001);
    public static PIDController transY = new PIDController(0.06, 0, 0.001);
    public static PIDController rotation = new PIDController(0.015, 0, 0.005);
    public static ProfiledPIDController rotation2 =
        new ProfiledPIDController(
            0.025, 0, 0.001, new TrapezoidProfile.Constraints(13.200, 36.366));
    public static HolonomicDriveController holonomicController =
        new HolonomicDriveController(transX, transY, rotation2);

    static {
      rotation.enableContinuousInput(-Math.PI, Math.PI);
      rotation.setTolerance(Math.toRadians(2));
      rotation2.enableContinuousInput(-Math.PI, Math.PI);
      rotation2.setTolerance(Math.toRadians(2));
      holonomicController.setTolerance(new Pose2d(0.02, 0.02, Rotation2d.fromDegrees(1)));
    }
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
