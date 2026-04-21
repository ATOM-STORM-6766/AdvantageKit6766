package frc.robot.subsystems.feeder;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface FeederIO {
  @AutoLog
  class FeederInputs {
    public AngularVelocity shooterVelocity = RPM.of(0.0);
    public Voltage shooterAppliedVolts = Volts.of(0.0);
    public Current shooterTorqueCurrent = Amps.of(0.0);

    public AngularVelocity intakeVelocity = RPM.of(0.0);
    public Voltage intakeAppliedVolts = Volts.of(0.0);
    public Current intakeTorqueCurrent = Amps.of(0.0);
  }

  default void readInputs(FeederInputs inputs) {}

  default void setShooterVelocity(AngularVelocity velocity) {}

  default void setIntakeVelocity(AngularVelocity velocity) {}

  default void stop() {}

  static FeederIO getIO() {
    switch (frc.robot.Constants.currentMode) {
      case REAL:
        return new FeederIOTalonFX();
      default:
        return new FeederIOSim();
    }
  }
}
