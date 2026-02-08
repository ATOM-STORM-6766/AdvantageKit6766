package frc.robot.subsystems.flywheel;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.RobotBase;

public class FlywheelConstants {
  // CAN IDs (three motors, shared config)
  public static final int kFlywheelMotor0CanID = 0;
  public static final int kFlywheelMotor1CanID = 1;
  public static final int kFlywheelMotor2CanID = 2;

  // Gear ratio: motor rotations per output rotation
  public static final double kFlywheelGearRatio = 1.0;

  // Velocity (RPS)
  public static final double kFlywheelIdleVelocityRps = 0.0;
  public static final double kFlywheelVelocityToleranceRps = 1.0;

  public static TalonFXConfiguration getTalonFXConfig() {
    var config = new TalonFXConfiguration();

    // Motor output
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    // PID + Feedforward configuration
    config.Slot0.kP = 9;
    config.Slot0.kI = 0.0;
    config.Slot0.kD = 0.0;
    config.Slot0.kS = 8;
    config.Slot0.kV = 0.699999988079071; // Ampere per RPS //volts per RPS
    config.Slot0.kA = 0.0;

    // Current limits (real robot only)
    if (RobotBase.isReal()) {
      config.CurrentLimits.StatorCurrentLimit = 80.0;
      config.CurrentLimits.StatorCurrentLimitEnable = true;
      config.CurrentLimits.SupplyCurrentLimit = 60.0;
      config.CurrentLimits.SupplyCurrentLimitEnable = true;

      config.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.02;
      config.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.05;
    }

    return config;
  }
}
