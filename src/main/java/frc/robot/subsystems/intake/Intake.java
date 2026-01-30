package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  public Intake(IntakeIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);
  }

  public Command setIntakeVelocityCommand(double voltage) {
    return runOnce(() -> io.setIntakeVelocity(voltage)).withName("Intake Set Intake Velocity");
  }

  public Command setFeedVelocityCommand(double velocityRadPerSec) {
    return runOnce(() -> io.setFeedVelocity(velocityRadPerSec))
        .withName("Intake Set Feed Velocity");
  }

  public Command setFeedIntakeVelocityCommand(double intakeVoltage, double feedVelocityRadPerSec) {
    return runOnce(
            () -> {
              io.setIntakeVelocity(intakeVoltage);
              io.setFeedVelocity(feedVelocityRadPerSec);
            })
        .withName("Intake Set Feed and Intake Velocity");
  }
}
