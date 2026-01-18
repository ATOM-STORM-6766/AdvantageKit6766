package frc.robot.subsystems.turret;

import org.littletonrobotics.junction.AutoLog;

public interface TurretIO {
  @AutoLog
  class TurretInputs {
    // Turret variables
    public double positionRad = 0.0;
    public double positionDegrees = 0.0;
    public double velocityRadPerSec = 0.0;
    public double velocityDegreesPerSec = 0.0;

    public double rotorPosition = 0.0;
    public double rotorPositionRadians = 0.0;
    public double absoluteEncoderPosition = 0.0;
    public double absoluteEncoderPositionRadians = 0.0;
    public double absoluteEncoder2Position = 0.0;
    public double absoluteEncoder2PositionRadians = 0.0;
    public double appliedVolts = 0.0;
    public double currentStatorAmps = 0.0;
    public double currentSupplyAmps = 0.0;
  }

  default void readInputs(TurretInputs inputs) {}

  default void setPositionSetpoint(double radiansFromCenter, double radsPerSecond) {}

  default void setOpenloopVoltage(double voltage) {}

  default void setRotorPosition(double turretPositionRadians) {}
}
