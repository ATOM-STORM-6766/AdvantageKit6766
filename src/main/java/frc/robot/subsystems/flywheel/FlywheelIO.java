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
    public AngularVelocity velocity0 = RPM.of(0.0);
    public Voltage appliedVolts0 = Volts.of(0.0);
    public Current currentTorqueAmps0 = Amps.of(0);

    public AngularVelocity velocity1 = RPM.of(0.0);
    public Voltage appliedVolts1 = Volts.of(0.0);
    public Current currentTorqueAmps1 = Amps.of(0);

    public AngularVelocity velocity2 = RPM.of(0.0);
    public Voltage appliedVolts2 = Volts.of(0.0);
    public Current currentTorqueAmps2 = Amps.of(0);

    public AngularVelocity velocity3 = RPM.of(0.0);
    public Voltage appliedVolts3 = Volts.of(0.0);
    public Current currentTorqueAmps3 = Amps.of(0);
  }

  public void readInputs(FlywheelInputs inputs);

  public void setFlywheelVelocity(AngularVelocity velocity);

  public void stop();

  static FlywheelIO getIO() {
    switch (frc.robot.Constants.currentMode) {
      case REAL:
        return new FlywheelIOTalonFX();
      default:
        return new FlywheelIOSim();
    }
  }
}
