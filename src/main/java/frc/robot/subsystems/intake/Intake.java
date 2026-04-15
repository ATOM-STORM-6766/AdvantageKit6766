package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  enum State {
    UNINITIALIZED,
    INITIALIZED
  }

  public enum Position {
    DEPLOYED(IntakeConstants.kIntakeMaxPosition),
    STOWED(IntakeConstants.kIntakeMinPosition);

    private final Angle setpoint;

    Position(Angle setpoint) {
      this.setpoint = setpoint;
    }

    public Angle getSetpoint() {
      return setpoint;
    }
  }

  private State state = State.UNINITIALIZED;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
  private final IntakeIO io;
  private final Debouncer calibrationCurrentDebouncer =
      new Debouncer(IntakeConstants.kCalibrationDebounceTimeSec, Debouncer.DebounceType.kBoth);

  public Intake() {
    this.io = IntakeIO.getIO();
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
                      io.setIntakeSensorPosition(IntakeConstants.kIntakeMinPosition);
                      state = State.INITIALIZED;
                      Logger.recordOutput("Intake/API/initailized", true);
                    }),
            Commands.none(),
            () -> state == State.UNINITIALIZED)
        .withName("Intake Reset to Limit");
  }

  public Command setIntakeVelocityCommand(Supplier<Voltage> voltageSupplier) {
    return runOnce(() -> io.setIntakeVelocity(voltageSupplier.get()))
        .withName("Intake Set Intake Velocity");
  }

  public Command setPosCommand(Position targetPosition) {
    return Commands.runOnce(() -> setIntakePositionImpl(targetPosition.getSetpoint()))
        .withName("Intake Set Position");
  }

  public Command stopCommand() {
    return runOnce(() -> io.stop()).withName("Intake Stop");
  }

  public Command setSlowStowCommand() {
    Angle target = Position.STOWED.getSetpoint();
    return run(() -> {
          setIntakePositionWithVelocityImpl(target, IntakeConstants.kSlowStowVelocityRPS);
          io.setIntakeVelocity(Volts.of(5));
        })
        .until(() -> isAtPosition(target))
        .andThen(runOnce(() -> io.stop()))
        .withName("Intake Slow Stow");
  }

  private void setIntakePositionImpl(Angle targetPositionRotation) {
    Logger.recordOutput("Intake/API/rotationSetpoint", targetPositionRotation);
    io.setIntakePosition(targetPositionRotation);
  }

  private void setIntakePositionWithVelocityImpl(Angle targetPosition, double velocityRPS) {
    Logger.recordOutput("Intake/API/rotationSetpoint", targetPosition);
    Logger.recordOutput("Intake/API/velocitySetpointRPS", velocityRPS);
    double direction = targetPosition.gt(inputs.intakePositionRotation) ? 1.0 : -1.0;
    io.setIntakePositionWithVelocity(targetPosition, velocityRPS * direction);
  }

  private boolean isAtPosition(Angle targetPosition) {
    return Math.abs(inputs.intakePositionRotation.in(Rotations) - targetPosition.in(Rotations))
        <= IntakeConstants.kPositionTolerance.in(Rotations);
  }

  private boolean isCalibrationStalled() {
    boolean currentOverThreshold =
        calibrationCurrentDebouncer.calculate(
            inputs.positionStatorAmps.abs(Amps) >= IntakeConstants.kCalibrationCurrentThreshold);
    return inputs.positionVelocity.abs(RadiansPerSecond)
            <= IntakeConstants.kCalibrationVelocityThresholdRadPerSec
        && currentOverThreshold;
  }
}
