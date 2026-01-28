package frc.robot.subsystems.intake;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  @AutoLog
  public static class IntakeIOInputs {
    public Rotation2d position = new Rotation2d();
    public double currentAmps = 0.0;
    public double voltageVolts = 0.0;
  }

  public void updateInputs(IntakeIOInputs inputs);

  public void setPosition(Rotation2d position);
}
