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
  public static final int rollerFollowerMotorID = 24;

  // Gear ratio: motor rotations per output rotation
  public static final double kPositionGearRatio = 66.0 / 14.0 * 18.0 / 18.0;

  public static final double kPositionGearRadius = 0.0254 / 2.0;
  public static final double kPositionMetersPerMechanismRotation =
      Math.PI * 2.0 * kPositionGearRadius;

  public static final double kIntakeMinMechanismRotations = 0.0;
  public static final double kIntakeMaxMechanismRotations = 2.0;
  public static final Distance kIntakeMinPosition =
      Meters.of(kIntakeMinMechanismRotations * kPositionMetersPerMechanismRotation);
  public static final Distance kIntakeMaxPosition =
      Meters.of(kIntakeMaxMechanismRotations * kPositionMetersPerMechanismRotation);

  public static final double kCalibrationVoltage = -5.0;
  public static final double kCalibrationCurrentThreshold = 10.0;
  public static final double kCalibrationVelocityThresholdRadPerSec = 0.1;
  public static final double kCalibrationDebounceTimeSec = 0.1;

  public static TalonFXConfiguration getRollerConfig() {
    var config = new TalonFXConfiguration();

    // Motor output
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    if (RobotBase.isReal()) {
      config.TorqueCurrent.PeakForwardTorqueCurrent = 120.0;
      config.TorqueCurrent.PeakReverseTorqueCurrent = -120.0;
      config.CurrentLimits.SupplyCurrentLimit = 40.0;
      config.CurrentLimits.SupplyCurrentLimitEnable = true;
    }

    return config;
  }

  public static TalonFXConfiguration getPositionConfig() {
    var config = new TalonFXConfiguration();

    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
    config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = kIntakeMaxMechanismRotations;
    config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = kIntakeMinMechanismRotations;

    config.Feedback.SensorToMechanismRatio = kPositionGearRatio;

    config.Slot0.kP = 30.0;
    config.Slot0.kD = 0.0;
    config.Slot0.kS = 14.0;
    config.Slot0.kV = 0.0;
    config.Slot0.kA = 0.0;
    config.MotionMagic.MotionMagicCruiseVelocity = 12.0;
    config.MotionMagic.MotionMagicAcceleration = 12.0;
    config.MotionMagic.MotionMagicJerk = 40.0;
    config.MotionMagic.MotionMagicExpo_kV = 0.5;
    config.MotionMagic.MotionMagicExpo_kA = 0.2;

    if (RobotBase.isReal()) {
      config.TorqueCurrent.PeakForwardTorqueCurrent = 50.0;
      config.TorqueCurrent.PeakReverseTorqueCurrent = -50.0;
    }

    return config;
  }
}
