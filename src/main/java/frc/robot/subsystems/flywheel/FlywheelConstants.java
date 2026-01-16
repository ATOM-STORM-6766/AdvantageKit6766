package frc.robot.subsystems.flywheel;

import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.RobotBase;

public class FlywheelConstants {
  // CAN IDs (placeholder values - update with actual hardware IDs)
  public static final int kFlywheelMotorCanID = 0;

  // Gear ratio: motor rotations per output rotation
  // Example: For a 2:1 gearbox, set this to 2.0
  public static final double kFlywheelGearRatio = 1.0; // TODO: Update with actual ratio

  // Velocity targets (in RPM)
  public static final double kFlywheelIdleVelocityRPM = 0.0;
  public static final double kFlywheelMaxVelocityRPM = 6000.0; // TODO: Update based on testing
  public static final double kFlywheelVelocityTolerance = 50.0; // RPM

  // PID + Feedforward gains for velocity control
  public static final double kS = 0.0; // Static friction voltage
  public static final double kP = 0.1; // Proportional gain
  public static final double kI = 0.0; // Integral gain
  public static final double kD = 0.0; // Derivative gain
  public static final double kV = 0.12; // Velocity feedforward (volts per RPS)
  public static final double kA = 0.0; // Acceleration feedforward

  // Current limits
  public static final double kStatorCurrentLimit = 80.0;
  public static final boolean kStatorCurrentLimitEnable = true;
  public static final double kSupplyCurrentLimit = 60.0;
  public static final boolean kSupplyCurrentLimitEnable = true;

  // Ramp rates
  public static ClosedLoopRampsConfigs makeClosedLoopRampConfig() {
    var config = new ClosedLoopRampsConfigs();
    config.VoltageClosedLoopRampPeriod = 0.02;
    return config;
  }

  public static OpenLoopRampsConfigs makeOpenLoopRampConfig() {
    var config = new OpenLoopRampsConfigs();
    config.VoltageOpenLoopRampPeriod = 0.05;
    return config;
  }

  public static TalonFXConfiguration getTalonFXConfig() {
    var config = new TalonFXConfiguration();

    // Motor output
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    // PID + Feedforward configuration
    config.Slot0.kS = kS;
    config.Slot0.kP = kP;
    config.Slot0.kI = kI;
    config.Slot0.kD = kD;
    config.Slot0.kV = kV;
    config.Slot0.kA = kA;

    // Current limits (real robot only)
    if (RobotBase.isReal()) {
      config.CurrentLimits.StatorCurrentLimit = kStatorCurrentLimit;
      config.CurrentLimits.StatorCurrentLimitEnable = kStatorCurrentLimitEnable;
      config.CurrentLimits.SupplyCurrentLimit = kSupplyCurrentLimit;
      config.CurrentLimits.SupplyCurrentLimitEnable = kSupplyCurrentLimitEnable;
      config.ClosedLoopRamps = makeClosedLoopRampConfig();
      config.OpenLoopRamps = makeOpenLoopRampConfig();
    }

    return config;
  }
}
