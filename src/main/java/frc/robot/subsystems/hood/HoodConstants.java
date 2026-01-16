package frc.robot.subsystems.hood;

import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;

public class HoodConstants {
  // CAN IDs (placeholder values - update with actual hardware IDs)
  public static final int kHoodMotorCanID = 2;
  public static final String kHoodCanBus = "rio";

  // Gear ratio: motor rotations per hood mechanism rotation
  public static final double kHoodGearRatio = 1; // TODO: Update with actual ratio

  // Rotation limits (in radians from center)
  public static final double kHoodMinPositionRadians = 0.0; // 0 degrees
  public static final double kHoodMaxPositionRadians = Math.toRadians(60.0); // 60 degrees

  // PID + Feedforward gains
  public static final double kS = 0.18; // Static friction voltage
  public static final double kP = 8.0; // Proportional gain
  public static final double kD = 0.1; // Derivative gain
  public static final double kV = 0.116; // Velocity feedforward
  public static final double kA = 0.0001 * 12.0; // Acceleration feedforward

  // Motion Magic parameters
  public static final double kMotionMagicAcceleration = 1000.0; // rotations/sec^2
  public static final double kMotionMagicCruiseVelocity = 100.0; // rotations/sec

  // Current limits
  public static final double kStatorCurrentLimit = 50.0;
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
