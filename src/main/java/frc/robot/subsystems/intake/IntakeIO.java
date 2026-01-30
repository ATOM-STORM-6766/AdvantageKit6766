package frc.robot.subsystems.intake;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  @AutoLog
  public static class IntakeIOInputs {
    public Rotation2d intakeRotation = new Rotation2d();
    public double intakeVelocity = 0.0;
    public double feedVelocity = 0.0;
  }

  public void updateInputs(IntakeIOInputs inputs);

  public void setIntakePosition(Rotation2d position);

  public void setIntakeVelocity(double voltage);

  public void setFeedVelocity(double velocityRadPerSec);

  public default void stop() {
    setIntakeVelocity(0.0);
    setFeedVelocity(0.0);
  }
}
