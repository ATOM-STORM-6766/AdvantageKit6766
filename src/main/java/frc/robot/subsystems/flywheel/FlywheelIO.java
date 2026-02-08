package frc.robot.subsystems.flywheel;

import org.littletonrobotics.junction.AutoLog;

public interface FlywheelIO {
  @AutoLog
  class FlywheelInputs {
    public double velocityRps0 = 0.0;
    public double appliedVolts0 = 0.0;
    public double currentStatorAmps0 = 0.0;
    public double currentSupplyAmps0 = 0.0;

    public double velocityRps1 = 0.0;
    public double appliedVolts1 = 0.0;
    public double currentStatorAmps1 = 0.0;
    public double currentSupplyAmps1 = 0.0;

    public double velocityRps2 = 0.0;
    public double appliedVolts2 = 0.0;
    public double currentStatorAmps2 = 0.0;
    public double currentSupplyAmps2 = 0.0;
  }

  public record FlywheelSetpointRps(double rps0, double rps1, double rps2) {}

  default void readInputs(FlywheelInputs inputs) {}

  default void setVelocity(double rps0, double rps1, double rps2) {}
}
