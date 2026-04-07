package frc.robot.subsystems.hood;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.RobotBase;

public class HoodConstants {
  // CAN IDs
  public static final int kHoodMotorCanID = 10;

  // Gear ratio: motor rotations per output rotation
  public static final double kHoodGearRatio = 105;

  public static final Angle kHoodMinPosition = Degrees.of(14.0);
  public static final Angle kHoodMaxPosition = Degrees.of(39.0);

  // Calibration parameters for limit-based reset
  public static final double kCalibrationVoltage = -1.5;
  public static final double kCalibrationCurrentThreshold = 10.0; // Amperes
  public static final double kCalibrationVelocityThresholdRadPerSec = 0.1;
  public static final double kCalibrationDebounceTimeSec = 0.1;

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
    config.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
        Units.radiansToRotations(kHoodMaxPosition.in(Radians)) * kHoodGearRatio;

    // Current limits (real robot only)
    if (RobotBase.isReal()) {
      config.CurrentLimits.StatorCurrentLimit = 30.0;
      config.CurrentLimits.StatorCurrentLimitEnable = true;

      config.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.0;
      config.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.02;
    }

    return config;
  }
}
