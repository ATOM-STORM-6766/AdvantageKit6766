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

    config.robotCenterToTurret = new Translation3d(-0.235, 0.0, 0.63);

    config.hoodPitchRadiansMap.put(1.51, Rotation2d.fromDegrees(25.0));
    config.hoodPitchRadiansMap.put(2.47, Rotation2d.fromDegrees(28.0));
    config.hoodPitchRadiansMap.put(2.87, Rotation2d.fromDegrees(30.0));
    config.hoodPitchRadiansMap.put(3.52, Rotation2d.fromDegrees(35.0));
    config.hoodPitchRadiansMap.put(4.05, Rotation2d.fromDegrees(37.0));
    config.hoodPitchRadiansMap.put(4.90, Rotation2d.fromDegrees(38.0));

    config.flywheelRpsMap.put(1.51, 50.0);
    config.flywheelRpsMap.put(2.47, 50.0);
    config.flywheelRpsMap.put(2.87, 55.0);
    config.flywheelRpsMap.put(3.52, 55.0);
    config.flywheelRpsMap.put(4.05, 55.0);
    config.flywheelRpsMap.put(4.90, 57.0);

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
