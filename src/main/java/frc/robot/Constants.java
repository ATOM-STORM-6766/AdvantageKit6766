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

import com.ctre.phoenix6.CANBus;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.RobotBase;
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

  private static ShooterConfig createShooterConfig() {
    ShooterConfig config = new ShooterConfig();

    config.robotCenterToTurret = new Translation3d(-0.19685, 0.0, 0.44);

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

    config.flywheelRpsMap.put(1.45, 30.0);
    config.flywheelRpsMap.put(1.75, 40.0);
    config.flywheelRpsMap.put(2.15, 41.0);
    config.flywheelRpsMap.put(2.50, 42.0);
    config.flywheelRpsMap.put(2.84, 43.0);
    config.flywheelRpsMap.put(3.15, 44.0);
    config.flywheelRpsMap.put(3.58, 45.0);
    config.flywheelRpsMap.put(4.16, 46.0);
    config.flywheelRpsMap.put(4.43, 47.0);
    config.flywheelRpsMap.put(5.28, 53.0);

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
}
