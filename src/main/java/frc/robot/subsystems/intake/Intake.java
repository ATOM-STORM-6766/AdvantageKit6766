package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  enum State {
    UNINITIALIZED,
    INITIALIZED
  }

  public enum Position {
    DEPLOYED(Meters.of(0.0)),
    STOWED(Meters.of(0.3));

    private final Distance setpoint;

    Position(Distance setpoint) {
      this.setpoint = setpoint;
    }

    public Distance getSetpoint() {
      return setpoint;
    }
  }

  private State state = State.UNINITIALIZED;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
  private final IntakeIO io;
  private final Debouncer calibrationCurrentDebouncer = new Debouncer(IntakeConstants.kCalibrationDebounceTimeSec,
      Debouncer.DebounceType.kBoth);

  public Intake(IntakeIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.readInputs(inputs);
    Logger.processInputs("Intake", inputs);
  }

  public boolean isInitialized() {
    return state == State.INITIALIZED;
  }

  public Command resetToLimitCommand() {
    return Commands.either(
        run(() -> io.setPositionVoltage(Volts.of(IntakeConstants.kCalibrationVoltage)))
            .until(this::isCalibrationStalled)
            .andThen(
                () -> {
                  io.setPositionVoltage(Volts.of(0));
                  io.setIntakePosition(IntakeConstants.kIntakeMinPosition);
                  state = State.INITIALIZED;
                  Logger.recordOutput("Intake/API/initailized", true);
                }),
        Commands.none(),
        () -> state == State.UNINITIALIZED)
        .withName("Intake Reset to Limit");
  }

  public Command setIntakeVelocityCommand(Voltage voltage) {
    return runOnce(() -> io.setIntakeVelocity(voltage)).withName("Intake Set Intake Velocity");
  }

  public Command setPosCommand(Position targetPosition) {
    return Commands.runOnce(() -> setIntakePositionImpl(targetPosition.getSetpoint()))
        .withName("Intake Set Position");
  }

  public Command stopCommand() {
    return runOnce(() -> io.stop()).withName("Intake Stop");
  }

  private void setIntakePositionImpl(Distance targetPositionDistance) {
    Logger.recordOutput("Intake/API/setpoint", targetPositionDistance);
    io.setIntakePosition(targetPositionDistance);
  }

  private boolean isCalibrationStalled() {
    boolean currentOverThreshold = calibrationCurrentDebouncer.calculate(
        inputs.positionStatorAmps.abs(Amps) >= IntakeConstants.kCalibrationCurrentThreshold);
    return inputs.positionVelocity.abs(RadiansPerSecond) <= IntakeConstants.kCalibrationVelocityThresholdRadPerSec
        && currentOverThreshold;
  }
}
