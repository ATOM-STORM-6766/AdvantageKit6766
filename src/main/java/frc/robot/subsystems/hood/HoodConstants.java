package frc.robot.subsystems.hood;

import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;

public class HoodConstants {
  // CAN IDs (placeholder values - update with actual hardware IDs)
  public static final int kHoodMotorCanID = 2;
  public static final String kHoodCanBus = "rio";

  // Gear ratio: motor rotations per output rotation
  // Example: For a 100:1 gearbox, set this to 100.0
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

  public static TalonFXConfiguration getTalonFXConfig() {
    var config = new TalonFXConfiguration();

    // Motor output
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    // Software limits
    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
        Units.radiansToRotations(kHoodMaxPositionRadians) * kHoodGearRatio;
    config.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
        Units.radiansToRotations(kHoodMinPositionRadians) * kHoodGearRatio;

    // PID + Feedforward
    config.Slot0.kS = kS;
    config.Slot0.kP = kP;
    config.Slot0.kD = kD;
    config.Slot0.kV = kV;
    config.Slot0.kA = kA;

    // Current limits (real robot only)
    if (RobotBase.isReal()) {
      config.CurrentLimits.StatorCurrentLimit = kStatorCurrentLimit;
      config.CurrentLimits.StatorCurrentLimitEnable = kStatorCurrentLimitEnable;
      config.ClosedLoopRamps = makeClosedLoopRampConfig();
      config.OpenLoopRamps = makeOpenLoopRampConfig();
    }

    // Motion Magic
    config.MotionMagic.MotionMagicAcceleration = kMotionMagicAcceleration;
    config.MotionMagic.MotionMagicCruiseVelocity = kMotionMagicCruiseVelocity;

    return config;
  }
}
