package frc.robot.subsystems.flywheel;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.RobotBase;

public class FlywheelConstants {
  // CAN IDs
  public static final int kFlywheelMotorCanID0 = 11;
  public static final int kFlywheelMotorCanID1 = 12;
  public static final int kFlywheelMotorCanID2 = 13;

  public static final int kLimitSwitchID0 = 0;
  public static final int kLimitSwitchID1 = 1;
  public static final int kLimitSwitchID2 = 2;

  public static final double kFlywheelBoost = 1.1; // A/rps

  public static final double kFlywheelBoostHoldTimeSec = 0.17;
  // Gear ratio: motor rotations per output rotation
  // Example: For a 2:1 gearbox, set this to 2.0
  public static final double kFlywheelGearRatio = 1.0;

  // Velocity targets (in RPM)
  public static final double kFlywheelIdleVelocityRPM = 0.0;
  public static final double kFlywheelVelocityTolerance = 50.0; // RPM

  public static TalonFXConfiguration getTalonFXConfig() {
    var config = new TalonFXConfiguration();

    // Motor output
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    // PID + Feedforward configuration 这里为原来的没铜轮的值
    config.Slot0.kP = 5.5;
    config.Slot0.kI = 0.0;
    config.Slot0.kD = 0.0;
    config.Slot0.kS = 3;
    config.Slot0.kV = 0.2; // Ampere per RPS //volts per RPS

    // Boost slot
    config.Slot1.kP = 0.0;
    config.Slot1.kI = 0.0;
    config.Slot1.kD = 0.01;
    config.Slot1.kS = 8;
    config.Slot1.kV = 0.4; // Ampere per RPS //volts per RPS

    config.MotionMagic.MotionMagicAcceleration = 100.0;
    config.MotionMagic.MotionMagicCruiseVelocity = 80.0;
    config.MotionMagic.MotionMagicJerk = 1000.0;

    config.Feedback.SensorToMechanismRatio = 1.2;

    // Current limits (real robot only)
    if (RobotBase.isReal()) {
      config.CurrentLimits.StatorCurrentLimit = 120.0;
      config.CurrentLimits.StatorCurrentLimitEnable = true;
      config.CurrentLimits.SupplyCurrentLimit = 80.0;
      config.CurrentLimits.SupplyCurrentLimitEnable = true;

      config.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.05;
      config.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.1;
    }

    return config;
  }
}
