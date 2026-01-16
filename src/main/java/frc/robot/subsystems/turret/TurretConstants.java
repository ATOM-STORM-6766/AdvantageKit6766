package frc.robot.subsystems.turret;

import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;

public class TurretConstants {
  // CAN IDs (placeholder values - update with actual hardware IDs)
  public static final int kTurretMotorCanID = 1;
  public static final int kTurretCanCoderID = 22;
  public static final String kTurretCanBus = "rio";

  // Gear ratio: motor rotations per turret rotation
  public static final double kTurretGearRatio = 1; // TODO: Update with actual ratio

  // CANcoder offset (in rotations)
  public static final double kTurretCancoderOffset = 0.0; // TODO: Calibrate on actual hardware

  // Rotation limits (in radians from center)
  public static final double kTurretMinPositionRadians = 0.0; // 0 degrees
  public static final double kTurretMaxPositionRadians = Math.toRadians(270.0); // 270 degrees

  // PID + Feedforward gains
  public static final double kS = 0.18; // Static friction voltage
  public static final double kP = 6.0; // Proportional gain
  public static final double kD = 0.1; // Derivative gain
  public static final double kV = 0.120; // Velocity feedforward
  public static final double kA = 0.0001 * 12.0; // Acceleration feedforward

  // Motion Magic parameters
  public static final double kMotionMagicJerk = 0.0;
  public static final double kMotionMagicAcceleration = 900.0; // rotations/sec^2
  public static final double kMotionMagicCruiseVelocity = 90.0; // rotations/sec

  // Current limits
  public static final double kStatorCurrentLimit = 150.0;
  public static final boolean kStatorCurrentLimitEnable = true;

  // Ramp rates
  public static ClosedLoopRampsConfigs makeClosedLoopRampConfig() {
    var config = new ClosedLoopRampsConfigs();
    config.VoltageClosedLoopRampPeriod = 0.01;
    return config;
  }

  public static OpenLoopRampsConfigs makeOpenLoopRampConfig() {
    var config = new OpenLoopRampsConfigs();
    config.VoltageOpenLoopRampPeriod = 0.02;
    return config;
  }
}
