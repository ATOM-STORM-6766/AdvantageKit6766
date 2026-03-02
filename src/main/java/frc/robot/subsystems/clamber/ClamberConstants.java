package frc.robot.subsystems.clamber;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public final class ClamberConstants {
  public static final int kClamberMotorId = 30;
  public static final double kGearRatio = 1.0;
  public static final NeutralModeValue kNeutralMode = NeutralModeValue.Brake;
  public static final InvertedValue kInverted = InvertedValue.Clockwise_Positive;

  public static final double kCurrentLimit = 40.0;
}
