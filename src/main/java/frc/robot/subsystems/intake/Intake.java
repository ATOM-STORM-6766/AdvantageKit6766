package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  enum State {
    UNINITIALIZED,
    INITIALIZED
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
                                Rotations.of(IntakeConstants.minRotation + 0.001))
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
                                Rotations.of(IntakeConstants.minRotation + 0.001))
                            < 0),
            stopCommand().andThen(Commands.waitSeconds(1)))
        .withName("Intake SysId");
  }

  public Command resetToLimitCommand() {
    return run(() -> io.setPositionVoltage(Volts.of(1)))
        .until(this::isCalibrationStalled)
        .andThen(
            () -> {
              io.setPositionVoltage(Volts.of(0));
              io.setCurrentPosition(IntakeConstants.maxRotation);
              state = State.INITIALIZED;
            })
        .withName("Intake Reset to Limit");
  }

  public Command setIntakeVelocityCommand(Voltage voltage) {
    return runOnce(() -> io.setIntakeVelocity(voltage)).withName("Intake Set Intake Velocity");
  }

  public Command setPosCommand(Supplier<Angle> targetAngle) {
    return Commands.runOnce(() -> setIntakePositionImpl(targetAngle.get()))
        .withName("Intake Set Position");
  }

  public Command testSinPositionCommand() {
    Timer timer = new Timer();
    return Commands.startRun(
            () -> {
              timer.reset();
              timer.start();
            },
            () -> {
              var t = timer.get();
              setIntakePositionImpl(Degrees.of(t % 0.5 < 0.25 ? Math.min(t / 4, 1) * 20 + 40 : 0));
            })
        .withName("Intake Test Sin Position"); // TODO
  }

  public Command stopCommand() {
    return runOnce(() -> io.stop()).withName("Intake Stop");
  }

  private void setIntakePositionImpl(Angle targetAngle) {
    Logger.recordOutput("Intake/API/setpoint", targetAngle);
    io.setIntakePosition(targetAngle);
  }

  private boolean isCalibrationStalled() {
    return inputs.positionVelocity.abs(RadiansPerSecond)
            < IntakeConstants.kCalibrationVelocityThresholdRadPerSec
        && inputs.positionCurrent.baseUnitMagnitude()
            > IntakeConstants.kCalibrationCurrentThreshold;
  }
}
