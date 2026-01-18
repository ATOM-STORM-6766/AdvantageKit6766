package frc.robot.subsystems.turret;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.RobotBase;

public class TurretConstants {
  // CAN IDs
  public static final int kTurretMotorCanID = 17;

  public static final int kAbsoluteEncoderDIO = 0;
  public static final int kAbsoluteEncoder2DIO = 1;

  // Gear ratio: motor rotor rotations per turret rotation
  // Example: For a 100:1 gearbox, set this to 100.0
  public static final double kTurretGearRatio = 100.0 / 10.0;

  // Gear ratio: absolute encoder rotations per turret rotation
  public static final double kTurretAbsoluteEncoderToTurretRatio = 100.0 / 11.0;
  public static final double kTurretAbsoluteEncoder2ToTurretRatio = 100.0 / 10.0;

  // CRT algorithm tolerance (radians)
  public static final double kCRTToleranceRadians = Math.toRadians(5.0);

  // Rotation limits (in radians from center)
  public static final double kTurretMinPositionRadians = Math.toRadians(-180.0);
  public static final double kTurretMaxPositionRadians = Math.toRadians(180.0);

  // Calibration parameters for limit-based reset
  public static final double kCalibrationVoltage = -1.5;
  public static final double kCalibrationCurrentThreshold = 45.0; // Amperes
  public static final double kCalibrationVelocityThresholdRadPerSec = 1.0; // radians/sec

  public static TalonFXConfiguration getTalonFXConfig() {
    var config = new TalonFXConfiguration();

    // Motor output
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    // PID + Feedforward
    config.Slot0.kS = 0.18;
    config.Slot0.kP = 3.0;
    config.Slot0.kD = 0.0;
    config.Slot0.kV = 0.120;
    config.Slot0.kA = 0.0001 * 12.0;

    // Motion Magic
    config.MotionMagic.MotionMagicJerk = 0.0;
    config.MotionMagic.MotionMagicAcceleration = 200.0; // rotations/sec^2
    config.MotionMagic.MotionMagicCruiseVelocity = 20.0; // rotations/sec

    // Current limits (real robot only)
    if (RobotBase.isReal()) {
      config.CurrentLimits.StatorCurrentLimit = 150.0;
      config.CurrentLimits.StatorCurrentLimitEnable = true;

      config.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.01;
      config.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.02;
    }

    return config;
  }
}
