package frc.robot.subsystems.flywheel;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface FlywheelIO {
  @AutoLog
  class FlywheelInputs {
    public AngularVelocity velocityFeeder = RPM.of(0.0);
    public Voltage appliedVoltsFeeder = Volts.of(0.0);
    public Current currentTorqueAmpsFeeder = Amps.of(0);

    public AngularVelocity velocity0 = RPM.of(0.0);
    public Voltage appliedVolts0 = Volts.of(0.0);
    public Current currentTorqueAmps0 = Amps.of(0);

    public AngularVelocity velocity1 = RPM.of(0.0);
    public Voltage appliedVolts1 = Volts.of(0.0);
    public Current currentTorqueAmps1 = Amps.of(0);

    public AngularVelocity velocity2 = RPM.of(0.0);
    public Voltage appliedVolts2 = Volts.of(0.0);
    public Current currentTorqueAmps2 = Amps.of(0);
  }

  public record FlywheelSetpoint(
      AngularVelocity motor0, AngularVelocity motor1, AngularVelocity motor2) {}

  public void updateInputs(FlywheelInputs inputs);

  public void setFlywheelVelocity(FlywheelSetpoint velocities);

  public void setFlywheelWithBoost(FlywheelSetpoint velocities, boolean[] isBoost);

  public void setFeederVelocity(AngularVelocity velocity);

  public void stop();
}
