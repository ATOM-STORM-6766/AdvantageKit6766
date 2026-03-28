package frc.robot.subsystems.intake;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.RobotBase;

public class IntakeConstants {
  public static final CANBus canBus = CANBus.roboRIO();

  public static final int intakeMotorID = 20;
  public static final int positionMotorID = 21;
  public static final double maxRotation = 0.314698 + 0.03;
  public static final double minRotation = 0.0;
  public static final double positionGearRatio =
      60.0 / 8 * 58 / 20 * 30 / 15; // 43.5 注意int和float的除号"/"区别

  public static final double kCalibrationVoltage = 1.0;
  public static final double kCalibrationCurrentThreshold = 2.0; // Amperes
  public static final double kCalibrationVelocityThresholdRadPerSec = 0.025;

  public static TalonFXConfiguration getTalonFXConfig() {
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

      config.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.01;
      config.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.02;
    }

    return config;
  }

  public static TalonFXConfiguration getPositionConfig() {
    var config = new TalonFXConfiguration();

    config.Feedback.SensorToMechanismRatio = positionGearRatio;
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
    config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = maxRotation;
    config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = minRotation;

    config.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
    config.Slot0.kP = 30.0;
    config.Slot0.kD = 0.0;
    config.Slot0.kS = 0.0;
    config.Slot0.kV = 0.0;
    config.Slot0.kA = 0.05;
    config.Slot0.kG = 0.3;
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
