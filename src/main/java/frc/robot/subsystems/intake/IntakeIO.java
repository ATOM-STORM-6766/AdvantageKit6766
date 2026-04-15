package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Amp;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  @AutoLog
  public static class IntakeIOInputs {
    public Distance intakePosition = Meters.of(0);
    public Angle intakePositionRotation = Rotations.of(0.0);
    public AngularVelocity positionVelocity = RotationsPerSecond.of(0.0);
    public Current positionStatorAmps = Amp.of(0.0);
    public Current positionSupplyAmps = Amp.of(0.0);
    public AngularVelocity intakeVelocity0 = RotationsPerSecond.of(0.0);
    public AngularVelocity intakeVelocity1 = RotationsPerSecond.of(0.0);
  }

  public void readInputs(IntakeIOInputs inputs);

  public void setIntakePosition(Angle position);

  public void setIntakeVelocity(Voltage voltage);

  public void setIntakePositionWithVelocity(Angle position, double velocityRPS);

  public void setPositionVoltage(Voltage voltage);

  public void setIntakeSensorPosition(Angle position);

  public default void stop() {
    setIntakeVelocity(Volts.of(0));
    setPositionVoltage(Volts.of(0));
  }

  static IntakeIO getIO() {
    switch (frc.robot.Constants.currentMode) {
      case REAL:
        return new IntakeIOTalonFX();
      case SIM:
        return new IntakeIOSim();
      default:
        return new IntakeIOTalonFX();
    }
  }
}
