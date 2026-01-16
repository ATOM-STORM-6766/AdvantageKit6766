package frc.robot.subsystems.flywheel;

import org.littletonrobotics.junction.AutoLog;

public interface FlywheelIO {
  @AutoLog
  class FlywheelInputs {
    public double velocityRPM = 0.0;
    public double appliedVolts = 0.0;
    public double currentStatorAmps = 0.0;
    public double currentSupplyAmps = 0.0;
  }

  default void readInputs(FlywheelInputs inputs) {}

  default void setVelocity(double velocityRPM, double ffVolts) {}

  default void setVoltage(double volts) {}

  default void stop() {}
}
