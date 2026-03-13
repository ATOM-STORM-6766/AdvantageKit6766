package frc.robot.subsystems.hood;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class Hood extends SubsystemBase {
  enum State {
    UNINITIALIZED,
    INITIALIZED
  }

  private State state = State.UNINITIALIZED;
  private final HoodInputsAutoLogged inputs = new HoodInputsAutoLogged();
  private final HoodIO io;
  private final Debouncer calibrationCurrentDebouncer =
      new Debouncer(HoodConstants.kCalibrationDebounceTimeSec, Debouncer.DebounceType.kBoth);

  public Hood(final HoodIO io) {
    this.io = io;
    setDefaultCommand(positionSetpointCommand(() -> HoodConstants.kHoodMaxPosition));
  }

  @Override
  public void periodic() {
    io.readInputs(inputs);
    Logger.processInputs("Hood", inputs);
  }

  public boolean isInitialized() {
    return state == State.INITIALIZED;
  }

  public Command positionSetpointCommand(Supplier<Angle> setpoint) {
    return run(() -> {
          Angle target = setpoint.get();
          setPositionSetpointImpl(target);
        })
        .withName("Hood positionSetpointCommand");
  }

  public Command waitForPosition(Supplier<Angle> setpoint, Angle tolerance) {
    return new WaitUntilCommand(
            () -> {
              Angle target = setpoint.get();
              return inputs.position.minus(target).abs(Radians) < tolerance.in(Radians);
            })
        .withName("Hood wait for position");
  }

  public Command resetToLimitCommand() {
    return run(() -> io.setOpenloopVoltage(Volts.of(HoodConstants.kCalibrationVoltage)))
        .until(this::isCalibrationStalled)
        .andThen(
            runOnce(
                () -> {
                  io.setOpenloopVoltage(Volts.of(0.0));
                  io.setRotorPosition(HoodConstants.kHoodMinPosition);
                  state = State.INITIALIZED;
                }))
        .withName("Hood Reset to Limit");
  }

  private void setPositionSetpointImpl(Angle setpoint) {
    Logger.recordOutput("Hood/API/setPositionSetpoint/setpoint", setpoint);
    io.setPositionSetpoint(setpoint);
  }

  private boolean isCalibrationStalled() {
    boolean currentOverThreshold =
        calibrationCurrentDebouncer.calculate(
            inputs.currentStatorAmps.abs(Amps) >= HoodConstants.kCalibrationCurrentThreshold);
    return inputs.velocity.abs(RadiansPerSecond)
            <= HoodConstants.kCalibrationVelocityThresholdRadPerSec
        && currentOverThreshold;
  }
}
