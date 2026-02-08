package frc.robot.subsystems.hood;

import org.littletonrobotics.junction.AutoLog;

public interface HoodIO {
  enum CalibrationState {
    NOT_CALIBRATED,
    CALIBRATING,
    CALIBRATED
  }

  @AutoLog
  class HoodInputs {
    public double positionRad = 0.0;
    public double positionDegrees = 0.0;
    public double velocityRadPerSec = 0.0;
    public double velocityDegreesPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double currentStatorAmps = 0.0;
    public double currentSupplyAmps = 0.0;
  }

  default void readInputs(HoodIO.HoodInputs inputs) {}

  default void setPositionSetpoint(double setpoint) {}

  default void setOpenloopVoltage(double voltage) {}

  default void setRotorPosition(double hoodPositionRadians) {}
}
