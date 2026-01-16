package frc.robot.subsystems.hood;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.signals.NeutralModeValue;
import java.util.Arrays;
import java.util.List;
import org.littletonrobotics.junction.AutoLog;

public interface HoodIO {
  @AutoLog
  class HoodInputs {
    public double positionRad = 0.0;
    public double positionRotations = 0.0;
    public double velocityRadPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double currentStatorAmps = 0.0;
    public double currentSupplyAmps = 0.0;
  }

  default List<BaseStatusSignal> getStatusSignals() {
    return Arrays.asList();
  }

  default void readInputs(HoodIO.HoodInputs inputs) {}

  default void update(final HoodIO.HoodInputs inputs) {}

  default void setNeutralMode(NeutralModeValue neutralMode) {}

  default void setPositionSetpoint(double radiansFromCenter, double radsPerSec) {}

  default void setDutyCycleOut(double percentOutput) {}

  default void resetZeroPoint() {}
}
