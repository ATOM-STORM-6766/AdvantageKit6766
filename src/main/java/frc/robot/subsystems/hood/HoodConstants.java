package frc.robot.subsystems.hood;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.RobotBase;

public class HoodConstants {
  // CAN IDs
  public static final int kHoodMotorCanID = 10;

  // Gear ratio: motor rotations per output rotation
  public static final double kHoodGearRatio = 30.0 / 12.0 * 167.0 / 10.0;

  public static final Angle kHoodMinPosition = Degrees.of(15.0);
  public static final Angle kHoodMaxPosition = Degrees.of(45.0);

  // Calibration parameters for limit-based reset
  public static final double kCalibrationVoltage = -1.5;
  public static final double kCalibrationCurrentThreshold = 10.0; // Amperes
  public static final double kCalibrationVelocityThresholdRadPerSec = 0.1;
  public static final double kCalibrationDebounceTimeSec = 0.1;

  public static TalonFXConfiguration getTalonFXConfig() {
    var config = new TalonFXConfiguration();

    // Motor output
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    config.Feedback.SensorToMechanismRatio = kHoodGearRatio;

    config.Slot0.kS = 0.18;
    config.Slot0.kP = 8.0;
    config.Slot0.kD = 0.1;
    config.Slot0.kV = 0.116;
    config.Slot0.kA = 0.0001 * 12.0;

    config.Slot0.kG = 5.0;
    config.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
    config.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;

    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
        Units.radiansToRotations(kHoodMaxPosition.in(Radians));

    if (RobotBase.isReal()) {
      config.TorqueCurrent.PeakForwardTorqueCurrent = 80.0;
      config.TorqueCurrent.PeakReverseTorqueCurrent = -80.0;
      config.CurrentLimits.SupplyCurrentLimit = 40.0;
      config.CurrentLimits.SupplyCurrentLimitEnable = false;
    }

    return config;
  }
}
