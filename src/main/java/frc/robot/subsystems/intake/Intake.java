package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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

  public Command resetToLimitCommand() {
    return run(() -> io.setPositionVoltage(1.0))
        .until(
            () -> {
              return inputs.positionCurrent > 12.0
                  && inputs.positionVelocity.abs(RadiansPerSecond) < 0.025;
            })
        .andThen(
            () -> {
              io.setPositionVoltage(0.0);
              io.setCurrentPosition(IntakeConstants.maxRotation);
            })
        .withName("Intake Reset Position");
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

  public Command setPosCommand(Angle targetAngle) {
    return Commands.runOnce(() -> io.setIntakePosition(targetAngle))
        .withName("Intake Set Position");
  }

  public Command testSinPositionCommand() {
    return Commands.run(
            () ->
                io.setIntakePosition(
                    Degrees.of(35.0 - 25.0 * Math.cos(Timer.getFPGATimestamp() * 6))))
        .withName("Intake Test Sin Position"); // TODO
  }

  public Command stopCommand() {
    return runOnce(() -> io.stop()).withName("Intake Stop");
  }
}
