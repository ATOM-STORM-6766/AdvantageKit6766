package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Amp;
import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotation;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  @AutoLog
  public static class IntakeIOInputs {
    public Angle intakePosition = Degree.of(0);
    public AngularVelocity positionVelocity = RotationsPerSecond.of(0.0);
    public Current positionCurrent = Amp.of(0.0);
    public AngularVelocity intakeVelocity = RotationsPerSecond.of(0.0);
    public AngularVelocity feedVelocity = RotationsPerSecond.of(0.0);
  }

  public void updateInputs(IntakeIOInputs inputs);

  public void setIntakePosition(Angle position);

  public void setIntakeVelocity(Voltage voltage);

  public void setFeedVelocity(AngularVelocity velocity);

  public void setPositionVoltage(Voltage voltage);

  public void setCurrentPosition(double positionRotations);

  public default void stop() {
    setIntakeVelocity(Volts.of(0));
    setFeedVelocity(RadiansPerSecond.of(0));
    setIntakePosition(Rotation.of(IntakeConstants.minRotation));
  }
}
