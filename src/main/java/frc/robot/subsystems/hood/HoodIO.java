package frc.robot.subsystems.hood;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface HoodIO {
  @AutoLog
  class HoodInputs {
    public Angle position = Degrees.of(0.0);
    public AngularVelocity velocity = Degrees.per(Second).of(0.0);
    public Voltage appliedVolts = Volts.of(0.0);
    public Current currentStatorAmps = Amps.of(0.0);
    public Current currentSupplyAmps = Amps.of(0.0);
  }

  default void readInputs(HoodIO.HoodInputs inputs) {}

  default void setPositionSetpoint(Angle setpoint) {}

  default void setOpenloopVoltage(Voltage voltage) {}

  default void setPosition(Angle hoodPosition) {}

  static HoodIO getIO() {
    switch (frc.robot.Constants.currentMode) {
      case REAL:
        return new HoodIOTalonFX();
      default:
        return new HoodIOSim();
    }
  }
}
