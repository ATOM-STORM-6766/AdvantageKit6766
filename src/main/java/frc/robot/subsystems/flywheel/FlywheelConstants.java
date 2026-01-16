package frc.robot.subsystems.flywheel;

import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;

public class FlywheelConstants {
  // CAN IDs (placeholder values - update with actual hardware IDs)
  public static final int kFlywheelMotorCanID = 0;

  // Gear ratio: motor rotations per flywheel rotation
  public static final double kFlywheelGearRatio = 1.0; // TODO: Update with actual ratio

  // Velocity targets (in RPM)
  public static final double kFlywheelIdleVelocityRPM = 0.0;
  public static final double kFlywheelMaxVelocityRPM = 6000.0; // TODO: Update based on testing
  public static final double kFlywheelVelocityTolerance = 50.0; // RPM

  // PID + Feedforward gains for velocity control
  public static final double kS = 0.0; // Static friction voltage
  public static final double kP = 0.1; // Proportional gain
  public static final double kI = 0.0; // Integral gain
  public static final double kD = 0.0; // Derivative gain
  public static final double kV = 0.12; // Velocity feedforward (volts per RPS)
  public static final double kA = 0.0; // Acceleration feedforward

  // Current limits
  public static final double kStatorCurrentLimit = 80.0;
  public static final boolean kStatorCurrentLimitEnable = true;
  public static final double kSupplyCurrentLimit = 60.0;
  public static final boolean kSupplyCurrentLimitEnable = true;

  // Ramp rates
  public static ClosedLoopRampsConfigs makeClosedLoopRampConfig() {
    var config = new ClosedLoopRampsConfigs();
    config.VoltageClosedLoopRampPeriod = 0.02;
    return config;
  }

  public static OpenLoopRampsConfigs makeOpenLoopRampConfig() {
    var config = new OpenLoopRampsConfigs();
    config.VoltageOpenLoopRampPeriod = 0.05;
    return config;
  }
}
