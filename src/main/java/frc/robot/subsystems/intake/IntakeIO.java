package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Amp;
import static edu.wpi.first.units.Units.Degree;
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
  }

  public void updateInputs(IntakeIOInputs inputs);

  public void setIntakePosition(Angle position);

  public void setIntakeVelocity(Voltage voltage);

  public void setPositionVoltage(Voltage voltage);

  public void setPosition(double positionRotations);

  public void setPositionCurrent(Current current);

  public default void stop() {
    setIntakeVelocity(Volts.of(0));
    setPositionVoltage(Volts.of(0));
  }
}
