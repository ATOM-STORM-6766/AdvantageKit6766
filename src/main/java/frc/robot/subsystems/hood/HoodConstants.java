package frc.robot.subsystems.hood;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.RobotBase;

public class HoodConstants {
  // CAN IDs
  public static final int kHoodMotorCanID = 1;

  // Gear ratio: motor rotations per output rotation
  // Example: For a 100:1 gearbox, set this to 100.0
  public static final double kHoodGearRatio = 144;

  public static final double kHoodMinPositionRadians = Math.toRadians(42.0);
  public static final double kHoodMaxPositionRadians = Math.toRadians(72.0);

  // Calibration parameters for limit-based reset
  public static final double kCalibrationVoltage = -1.5;
  public static final double kCalibrationCurrentThreshold = 19.0; // Amperes
  public static final double kCalibrationVelocityThresholdRadPerSec = 0.1;

  public static TalonFXConfiguration getTalonFXConfig() {
    var config = new TalonFXConfiguration();

    // Motor output
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    // PID + Feedforward
    config.Slot0.kS = 0.18;
    config.Slot0.kP = 8.0;
    config.Slot0.kD = 0.1;
    config.Slot0.kV = 0.116;
    config.Slot0.kA = 0.0001 * 12.0;

    // Motion Magic
    config.MotionMagic.MotionMagicAcceleration = 100.0; // rotations/sec^2
    config.MotionMagic.MotionMagicCruiseVelocity = 10.0; // rotations/sec

    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = kHoodMaxPositionRadians * kHoodGearRatio;

    // Current limits (real robot only)
    if (RobotBase.isReal()) {
      config.CurrentLimits.StatorCurrentLimit = 50.0;
      config.CurrentLimits.StatorCurrentLimitEnable = true;

      config.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.01;
      config.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.02;
    }

    return config;
  }
}
