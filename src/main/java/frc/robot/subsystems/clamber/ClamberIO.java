package frc.robot.subsystems.clamber;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface ClamberIO {
  @AutoLog
  public static class ClamberInputs {
    public Angle position = Rotations.of(0.0);
    public AngularVelocity velocity = RotationsPerSecond.of(0.0);
    public Voltage appliedVolts = Volts.of(0.0);
    public Current supplyCurrent = Amps.of(0.0);
    public Current torqueCurrent = Amps.of(0.0);
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ClamberInputs inputs) {}

  /** Run the motor at the specified voltage. */
  public default void setVoltage(Voltage volts) {}

  /** Stop the motor. */
  public default void stop() {}
}
