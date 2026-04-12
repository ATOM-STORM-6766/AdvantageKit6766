package frc.robot.subsystems.feeder;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.RobotBase;

public class FeederConstants {
  public static final int kShooterFeedMotorCanID = 20;
  public static final int kIntakeFeedMotorCanID = 21;
  public static final int kIntakeFeedFollowerMotorCanID = 22;

  public static TalonFXConfiguration getShooterFeedTalonFXConfig() {
    var config = new TalonFXConfiguration();

    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    config.Slot0.kP = 5;
    config.Slot0.kI = 0.0;
    config.Slot0.kD = 0.0;
    config.Slot0.kS = 7.0;
    config.Slot0.kV = 0.0;

    if (RobotBase.isReal()) {
      config.TorqueCurrent.PeakForwardTorqueCurrent = 80.0;
      config.TorqueCurrent.PeakReverseTorqueCurrent = -80.0;
      config.CurrentLimits.SupplyCurrentLimit = 40.0;
      config.CurrentLimits.SupplyCurrentLimitEnable = true;
    }

    return config;
  }

  public static TalonFXConfiguration getIntakeFeedTalonFXConfig() {
    var config = new TalonFXConfiguration();

    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    config.Slot0.kP = 0.35;
    config.Slot0.kI = 0.0;
    config.Slot0.kD = 0.0005;
    config.Slot0.kS = 0.32;
    config.Slot0.kV = 0.13;

    if (RobotBase.isReal()) {
      config.TorqueCurrent.PeakForwardTorqueCurrent = 40.0;
      config.TorqueCurrent.PeakReverseTorqueCurrent = -40.0;
      config.CurrentLimits.SupplyCurrentLimit = 40.0;
      config.CurrentLimits.SupplyCurrentLimitEnable = true;
    }

    return config;
  }
}
