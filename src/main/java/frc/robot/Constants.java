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
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
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

  private static ShooterConfig createShooterConfig() {
    ShooterConfig config = new ShooterConfig();

    config.robotCenterToTurret = new Translation3d(-0.235, 0.0, 0.63);

    config.minRange = 1.50;
    config.maxRange = 5.00;

    config.minHoodPitchDegrees = HoodConstants.kHoodMinPosition.in(Degree);
    config.maxHoodPitchDegrees = HoodConstants.kHoodMaxPosition.in(Degree);

    config.maxFlywheelRps = 100;

    config.hoodPitchRadiansMap.put(1.50, Rotation2d.fromDegrees(25.0));
    config.hoodPitchRadiansMap.put(1.90, Rotation2d.fromDegrees(27.0));
    config.hoodPitchRadiansMap.put(2.30, Rotation2d.fromDegrees(29.0));
    config.hoodPitchRadiansMap.put(2.70, Rotation2d.fromDegrees(30.0));
    config.hoodPitchRadiansMap.put(3.10, Rotation2d.fromDegrees(32.0));
    config.hoodPitchRadiansMap.put(3.50, Rotation2d.fromDegrees(33.0));
    config.hoodPitchRadiansMap.put(3.90, Rotation2d.fromDegrees(35.0));
    config.hoodPitchRadiansMap.put(4.30, Rotation2d.fromDegrees(36.0));
    config.hoodPitchRadiansMap.put(4.70, Rotation2d.fromDegrees(37.0));
    config.hoodPitchRadiansMap.put(5.00, Rotation2d.fromDegrees(37.5));

    config.flywheelRpsMap.put(1.50, 47.0);
    config.flywheelRpsMap.put(1.90, 58.0);
    config.flywheelRpsMap.put(2.30, 51.0);
    config.flywheelRpsMap.put(2.70, 54.0);
    config.flywheelRpsMap.put(3.10, 55.0);
    config.flywheelRpsMap.put(3.50, 58.0);
    config.flywheelRpsMap.put(3.90, 59.0);
    config.flywheelRpsMap.put(4.30, 61.0);
    config.flywheelRpsMap.put(4.70, 62.5);
    config.flywheelRpsMap.put(5.00, 63.5);

    config.timeOfFlightSecondsMap.put(1.50, 0.83);
    config.timeOfFlightSecondsMap.put(1.90, 0.86);
    config.timeOfFlightSecondsMap.put(2.30, 0.89);
    config.timeOfFlightSecondsMap.put(2.70, 1.03);
    config.timeOfFlightSecondsMap.put(3.10, 1.13);
    config.timeOfFlightSecondsMap.put(3.50, 1.09);
    config.timeOfFlightSecondsMap.put(3.90, 1.18);
    config.timeOfFlightSecondsMap.put(4.30, 1.20);
    config.timeOfFlightSecondsMap.put(4.70, 1.22);
    config.timeOfFlightSecondsMap.put(5.00, 1.23);

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
