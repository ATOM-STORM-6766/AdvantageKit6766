package frc.robot.subsystems.flywheel;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.RobotBase;

public class FlywheelConstants {
  // CAN IDs
  public static final int kFlywheelMotorCanID0 = 11;
  public static final int kFlywheelMotorCanID1 = 12;
  public static final int kFlywheelMotorCanID2 = 14;
  public static final int kFlywheelMotorCanID3 = 13;

  // Gear ratio: motor rotations per output rotation
  public static final double kFlywheelGearRatio = 32.0 / 24.0;

  public static TalonFXConfiguration getTalonFXConfig() {
    var config = new TalonFXConfiguration();

    // Motor output
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    config.Feedback.SensorToMechanismRatio = kFlywheelGearRatio;

    config.Slot0.kP = 9.0;
    config.Slot0.kI = 0.0;
    config.Slot0.kD = 0.0;
    config.Slot0.kS = 2.5;
    config.Slot0.kV = 0.1;

    config.MotionMagic.MotionMagicCruiseVelocity = 100.0;
    config.MotionMagic.MotionMagicAcceleration = 2000.0;

    if (RobotBase.isReal()) {
      config.TorqueCurrent.PeakForwardTorqueCurrent = 80.0;
      config.TorqueCurrent.PeakReverseTorqueCurrent = -80.0;
      config.CurrentLimits.SupplyCurrentLimit = 40.0;
      config.CurrentLimits.SupplyCurrentLimitEnable = false;
    }

    return config;
  }
}
