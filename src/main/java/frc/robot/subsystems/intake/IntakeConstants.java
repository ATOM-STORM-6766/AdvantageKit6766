package frc.robot.subsystems.intake;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class IntakeConstants {
  public static final CANBus canBus = CANBus.roboRIO();

  public static final int intakeMotorID = 20;
  public static final int positionMotorID = 21;
  public static final int feedMotorID = 22;
  public static final double maxRotation = 0.314698;
  public static final double minRotation = 0.0;

  public static TalonFXConfiguration getTalonFXConfig() {
    var config = new TalonFXConfiguration();

    // Motor output
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    return config;
  }

  public static TalonFXConfiguration getPositionConfig() {
    var config = new TalonFXConfiguration();

    config.Feedback.SensorToMechanismRatio =
        60.0 / 8 * 58 / 20 * 30 / 15; // 43.5 注意int和float的除号"/"区别
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
    config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = maxRotation;
    config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = minRotation;

    config.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
    config.Slot0.kP = 7.0;
    config.Slot0.kI = 0.0;
    config.Slot0.kD = 0.0;
    config.Slot0.kS = 0.0;
    config.Slot0.kV = 0.0;
    config.Slot0.kA = 0.0;
    config.MotionMagic.MotionMagicExpo_kV = 0.12;
    config.MotionMagic.MotionMagicExpo_kA = 0.1;

    return config;
  }
}
