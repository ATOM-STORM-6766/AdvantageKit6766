package frc.robot.subsystems.turret;

import com.ctre.phoenix6.BaseStatusSignal;
import java.util.Arrays;
import java.util.List;
import org.littletonrobotics.junction.AutoLog;

public interface TurretIO {
  enum CalibrationState {
    NOT_CALIBRATED,
    CALIBRATING,
    CALIBRATED
  }

  @AutoLog
  class TurretInputs {
    public double positionRad = 0.0;
    public double velocityRadPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double currentStatorAmps = 0.0;
    public double currentSupplyAmps = 0.0;
    public CalibrationState calibrationState = CalibrationState.NOT_CALIBRATED;
  }

  default List<BaseStatusSignal> getStatusSignals() {
    return Arrays.asList();
  }

  default void readInputs(TurretInputs inputs) {}

  default void setPositionSetpoint(double radiansFromCenter, double radsPerSecond) {}

  default void startCalibration() {}
}
