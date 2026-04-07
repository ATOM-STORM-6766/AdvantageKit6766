package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Meters;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.RobotBase;

public class IntakeConstants {
  // CAN IDs
  public static final int intakeMotorID = 20;
  public static final int positionMotorID = 21;

  // Gear ratio: motor rotations per output rotation
  public static final double kPositionGearRatio = 60.0 / 8 * 58 / 20 * 30 / 15;
  public static final double kRollerGearRatio = 1.0;

  public static final double kPositionGearRadius = 0.03;
  public static final double kPositionMetersPerMechanismRotation =
      Math.PI * 2.0 * kPositionGearRadius;

  public static final double kIntakeMinMechanismRotations = 0.0;
  public static final double kIntakeMaxMechanismRotations = 10.0;
  public static final Distance kIntakeMinPosition =
      Meters.of(kIntakeMinMechanismRotations * kPositionMetersPerMechanismRotation);
  public static final Distance kIntakeMaxPosition =
      Meters.of(kIntakeMaxMechanismRotations * kPositionMetersPerMechanismRotation);
  public static final double kIntakeMinRotorRotations =
      kIntakeMinMechanismRotations * kPositionGearRatio;
  public static final double kIntakeMaxRotorRotations =
      kIntakeMaxMechanismRotations * kPositionGearRatio;

  public static final double kCalibrationVoltage = 1.0;
  public static final double kCalibrationCurrentThreshold = 2.0; // Amperes
  public static final double kCalibrationVelocityThresholdRadPerSec = 0.025;

  public static TalonFXConfiguration getRollerConfig() {
    var config = new TalonFXConfiguration();

    // Motor output
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    // PID + Feedforward configuration 这里为原来的没铜轮的值
    config.Slot0.kP = 0.2;
    config.Slot0.kI = 0.0;
    config.Slot0.kD = 0.0005;
    config.Slot0.kS = 0.32;
    config.Slot0.kV = 0.0; // Ampere per RPS //volts per RPS

    config.MotionMagic.MotionMagicAcceleration = 10.0;
    config.MotionMagic.MotionMagicCruiseVelocity = 8.0;
    config.MotionMagic.MotionMagicJerk = 100.0;

    if (RobotBase.isReal()) {
      config.CurrentLimits.StatorCurrentLimit = 40.0;
      config.CurrentLimits.StatorCurrentLimitEnable = true;

      config.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.0;
      config.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.02;
    }

    return config;
  }

  public static TalonFXConfiguration getPositionConfig() {
    var config = new TalonFXConfiguration();

    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
    config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = kIntakeMaxRotorRotations;
    config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = kIntakeMinRotorRotations;

    config.Slot0.kP = 30.0;
    config.Slot0.kD = 0.0;
    config.Slot0.kS = 0.0;
    config.Slot0.kV = 0.0;
    config.Slot0.kA = 0.05;
    config.MotionMagic.MotionMagicCruiseVelocity = 1.0;
    config.MotionMagic.MotionMagicAcceleration = 4.0;
    config.MotionMagic.MotionMagicJerk = 40.0;
    config.MotionMagic.MotionMagicExpo_kV = 0.5;
    config.MotionMagic.MotionMagicExpo_kA = 0.2;

    if (RobotBase.isReal()) {
      config.CurrentLimits.StatorCurrentLimit = 60.0;
      config.CurrentLimits.StatorCurrentLimitEnable = true;

      config.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.05;
      config.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.02;
    }

    return config;
  }
}
