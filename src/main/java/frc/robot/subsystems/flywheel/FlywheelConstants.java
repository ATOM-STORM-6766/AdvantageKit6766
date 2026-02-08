package frc.robot.subsystems.flywheel;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.RobotBase;

public class FlywheelConstants {
  // CAN IDs
  public static final int kFlywheelMotorCanID0 = 11;
  public static final int kFlywheelMotorCanID1 = 12;
  public static final int kFlywheelMotorCanID2 = 13;
  public static final int kShooterFeedMotorCanID = 14;

  public static final int kLimitSwitchID0 = 0;
  public static final int kLimitSwitchID1 = 1;
  public static final int kLimitSwitchID2 = 2;

  // Gear ratio: motor rotations per output rotation
  // Example: For a 2:1 gearbox, set this to 2.0
  public static final double kFlywheelGearRatio = 1.0; // TODO: Update with actual ratio

  // Velocity targets (in RPM)
  public static final double kFlywheelIdleVelocityRPM = 0.0;
  public static final double kFlywheelVelocityTolerance = 50.0; // RPM

  public static TalonFXConfiguration getTalonFXConfig() {
    var config = new TalonFXConfiguration();

    // Motor output
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

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
