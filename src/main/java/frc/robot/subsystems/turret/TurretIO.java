package frc.robot.subsystems.turret;

import com.ctre.phoenix6.BaseStatusSignal;
import edu.wpi.first.math.geometry.Rotation2d;
import java.util.Arrays;
import java.util.List;
import org.littletonrobotics.junction.AutoLog;

public interface TurretIO {
  @AutoLog
  class TurretInputs {
    public Rotation2d turretPositionAbsolute = new Rotation2d();
    public double positionRad = 0.0;
    public double velocityRadPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double currentStatorAmps = 0.0;
    public double currentSupplyAmps = 0.0;
    public double cancoderAbsolutePosition = 0.0;
  }

  default List<BaseStatusSignal> getStatusSignals() {
    return Arrays.asList();
  }

  // Read Inputs
  default void readInputs(TurretInputs inputs) {}

  // Set open loop duty cycle
  default void setOpenLoopDutyCycle(double dutyCycle) {}

  default void setPositionSetpoint(double radiansFromCenter, double radsPerSecond) {}
}
