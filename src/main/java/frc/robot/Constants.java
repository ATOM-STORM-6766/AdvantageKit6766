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
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.subsystems.turret.TurretConstants;
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

  // Turret offset from robot center (X: forward, Y: left, Z: height)
  public static final Translation3d ROBOT_CENTER_TO_TURRET = new Translation3d(0.2, 0.0, 0.3);

  public static final ShooterConfig SHOOTER_CONFIG = createShooterConfig();

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

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }
}
