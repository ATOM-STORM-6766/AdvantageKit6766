package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Amp;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
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

  private final SysIdRoutine sysIdRoutine;

  private State state = State.UNINITIALIZED;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
  private final IntakeIO io;

  public Intake(IntakeIO io) {
    this.io = io;
    this.sysIdRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.of(0.05).per(Seconds),
                Volts.of(0.285),
                Seconds.of(15.0),
                (state) -> Logger.recordOutput("intake/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> {
                  io.setPositionVoltage(voltage);
                  Logger.recordOutput("intake/Voltage", voltage.in(Volts));
                  Logger.recordOutput("intake/Position", inputs.intakePosition.in(Rotations));
                  Logger.recordOutput(
                      "intake/Velocity", inputs.positionVelocity.in(RotationsPerSecond));
                },
                null,
                this));
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);
  }

  public boolean isInitialized() {
    return state == State.INITIALIZED;
  }

  public Command runSysId() {
    return Commands.sequence(
            sysIdRoutine
                .dynamic(SysIdRoutine.Direction.kForward)
                .until(() -> inputs.intakePosition.compareTo(Rotations.of(0.25)) > 0),
            stopCommand().andThen(Commands.waitSeconds(1)),
            sysIdRoutine
                .dynamic(SysIdRoutine.Direction.kReverse)
                .until(
                    () ->
                        inputs.intakePosition.compareTo(
                                Rotations.of(IntakeConstants.minMechanismRotations + 0.001))
                            < 0),
            stopCommand().andThen(Commands.waitSeconds(1)),
            sysIdRoutine
                .quasistatic(SysIdRoutine.Direction.kForward)
                .until(() -> inputs.intakePosition.compareTo(Rotations.of(0.25)) > 0),
            stopCommand().andThen(Commands.waitSeconds(1)),
            sysIdRoutine
                .quasistatic(SysIdRoutine.Direction.kReverse)
                .until(
                    () ->
                        inputs.intakePosition.compareTo(
                                Rotations.of(IntakeConstants.minMechanismRotations + 0.001))
                            < 0),
            stopCommand().andThen(Commands.waitSeconds(1)))
        .withName("Intake SysId");
  }

  public Command resetToLimitCommand() {
    return Commands.either(
            run(() -> io.setPositionCurrent(Amp.of(3))) // io.setPositionVoltage(Volts.of(1))
                .until(this::isCalibrationStalled)
                .andThen(
                    () -> {
                      io.setPositionVoltage(Volts.of(0));
                      io.setPosition(IntakeConstants.maxMechanismRotations);
                      state = State.INITIALIZED;
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
    return inputs.positionVelocity.abs(RadiansPerSecond)
            < IntakeConstants.kCalibrationVelocityThresholdRadPerSec
        && inputs.positionCurrent.baseUnitMagnitude()
            > IntakeConstants.kCalibrationCurrentThreshold;
  }
}
