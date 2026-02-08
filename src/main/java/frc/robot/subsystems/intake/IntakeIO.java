package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Rotation;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  @AutoLog
  public static class IntakeIOInputs {
    public Rotation2d intakePosition = new Rotation2d();
    public AngularVelocity positionVelocity = RotationsPerSecond.of(0.0);
    public double positionCurrent = 0.0;
    public double intakeVelocity = 0.0;
    public double feedVelocity = 0.0;
  }

  public void updateInputs(IntakeIOInputs inputs);

  public void setIntakePosition(Angle position);

  public void setIntakeVelocity(double voltage);

  public void setFeedVelocity(double velocityRadPerSec);

  public void setPositionVoltage(double voltage);

  public void setCurrentPosition(double positionRotations);

  public default void stop() {
    setIntakeVelocity(0.0);
    setFeedVelocity(0.0);
    setIntakePosition(Rotation.of(IntakeConstants.minRotation));
  }
}
