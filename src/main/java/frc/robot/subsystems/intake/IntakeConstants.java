package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.RobotBase;

public class IntakeConstants {
  // CAN IDs
  public static final int intakeMotorID = 26;
  public static final int positionMotorID = 27;
  public static final int rollerFollowerMotorID = 28;

  // Gear ratio: motor rotations per output rotation
  public static final double kPositionGearRatio = 66.0 / 14.0 * 18.0 / 18.0;

  public static final double kPositionGearDiameter = 0.0254;
  public static final double kPositionMetersPerMechanismRotation = Math.PI * kPositionGearDiameter;

  public static final double kIntakeMinMechanismRotations = 0.0;
  public static final double kIntakeMaxMechanismRotations = 3.564;
  public static final Angle kIntakeMinPosition = Rotations.of(kIntakeMinMechanismRotations);
  public static final Angle kIntakeMaxPosition = Rotations.of(kIntakeMaxMechanismRotations);

  public static final double kSlowStowVelocityRPS = 3.0;
  public static final Angle kPositionTolerance = distanceToRotation(Meters.of(0.05));

  public static final double kCalibrationVoltage = -0.8;
  public static final double kCalibrationCurrentThreshold = 10.0;
  public static final double kCalibrationVelocityThresholdRadPerSec = 0.5;
  public static final double kCalibrationDebounceTimeSec = 0.1;

  public static Distance rotationToDistance(Angle position) {
    return Meters.of(position.in(Rotations) * kPositionMetersPerMechanismRotation);
  }

  public static Angle distanceToRotation(Distance position) {
    return Rotations.of(position.in(Meters) / kPositionMetersPerMechanismRotation);
  }

  public static TalonFXConfiguration getRollerConfig() {
    var config = new TalonFXConfiguration();

    // Motor output
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
    config.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;

    if (RobotBase.isReal()) {
      config.CurrentLimits.SupplyCurrentLimit = 40.0;
      config.CurrentLimits.SupplyCurrentLimitEnable = true;
    }

    return config;
  }

  public static TalonFXConfiguration getPositionConfig() {
    var config = new TalonFXConfiguration();

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
    config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = kIntakeMaxMechanismRotations;
    config.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;
    config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = kIntakeMinMechanismRotations;

    config.Feedback.SensorToMechanismRatio = kPositionGearRatio;

    config.Slot0.kP = 30.0;
    config.Slot0.kD = 0.0;
    config.Slot0.kS = 14.0;
    config.Slot0.kV = 0.0;
    config.Slot0.kA = 0.0;
    config.MotionMagic.MotionMagicCruiseVelocity = 10.0;
    config.MotionMagic.MotionMagicAcceleration = 1.0;

    if (RobotBase.isReal()) {
      config.TorqueCurrent.PeakForwardTorqueCurrent = 50.0;
      config.TorqueCurrent.PeakReverseTorqueCurrent = -50.0;
    }

    return config;
  }
}
