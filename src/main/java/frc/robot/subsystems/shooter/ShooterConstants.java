// Copyright 2021-2025 FRC 6766
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

package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

public class ShooterConstants {
  // Hardware configuration - CAN IDs
  public static final int HOOD_MOTOR_ID = 20;
  public static final int TURRET_MOTOR_ID = 21;
  public static final int FLYWHEEL_MOTOR_ID = 22;

  // Hardware configuration - Gear ratios
  public static final double HOOD_GEAR_RATIO = 1.0; // TODO: Set actual gear ratio
  public static final double TURRET_GEAR_RATIO = 1.0; // TODO: Set actual gear ratio
  public static final double FLYWHEEL_GEAR_RATIO = 1.0; // TODO: Set actual gear ratio

  // Physical limits - Hood
  public static final Rotation2d MIN_HOOD_ANGLE = Rotation2d.fromDegrees(0.0);
  public static final Rotation2d MAX_HOOD_ANGLE = Rotation2d.fromDegrees(60.0);

  // Physical limits - Turret
  public static final Rotation2d MIN_TURRET_ANGLE = Rotation2d.fromDegrees(-180.0);
  public static final Rotation2d MAX_TURRET_ANGLE = Rotation2d.fromDegrees(180.0);

  // Physical limits - Flywheel
  public static final double MAX_FLYWHEEL_RPM = 6000.0; // TODO: Set actual max RPM

  // PID gains - Hood
  public static final double HOOD_KP = 0.5; // TODO: Tune
  public static final double HOOD_KI = 0.0;
  public static final double HOOD_KD = 0.0;
  public static final double HOOD_KS = 0.0; // Static friction
  public static final double HOOD_KV = 0.0; // Velocity feedforward

  // PID gains - Turret
  public static final double TURRET_KP = 0.5; // TODO: Tune
  public static final double TURRET_KI = 0.0;
  public static final double TURRET_KD = 0.0;
  public static final double TURRET_KS = 0.0; // Static friction
  public static final double TURRET_KV = 0.0; // Velocity feedforward

  // PID gains - Flywheel
  public static final double FLYWHEEL_KP = 0.1; // TODO: Tune
  public static final double FLYWHEEL_KI = 0.0;
  public static final double FLYWHEEL_KD = 0.0;
  public static final double FLYWHEEL_KS = 0.0; // Static friction
  public static final double FLYWHEEL_KV = 0.0; // Velocity feedforward

  // Convergence thresholds
  public static final double HOOD_POSITION_TOLERANCE_RAD = Units.degreesToRadians(2.0);
  public static final double HOOD_VELOCITY_TOLERANCE_RAD_PER_SEC = Units.degreesToRadians(5.0);
  public static final double TURRET_POSITION_TOLERANCE_RAD = Units.degreesToRadians(2.0);
  public static final double TURRET_VELOCITY_TOLERANCE_RAD_PER_SEC = Units.degreesToRadians(5.0);
  public static final double FLYWHEEL_RPM_TOLERANCE = 50.0; // RPM

  // Limit detection parameters
  public static final double STALL_CURRENT_THRESHOLD = 20.0; // Amps
  public static final double STALL_VELOCITY_THRESHOLD = Units.degreesToRadians(1.0); // rad/s
  public static final double RESET_SPEED_RAD_PER_SEC = Units.degreesToRadians(10.0); // rad/s

  // Motion magic parameters - Hood
  public static final double HOOD_CRUISE_VELOCITY = Units.degreesToRadians(90.0); // rad/s
  public static final double HOOD_ACCELERATION = Units.degreesToRadians(180.0); // rad/s^2

  // Motion magic parameters - Turret
  public static final double TURRET_CRUISE_VELOCITY = Units.degreesToRadians(180.0); // rad/s
  public static final double TURRET_ACCELERATION = Units.degreesToRadians(360.0); // rad/s^2
}
