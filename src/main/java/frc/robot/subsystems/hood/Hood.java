package frc.robot.subsystems.hood;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class Hood extends SubsystemBase {

  private final HoodInputsAutoLogged inputs = new HoodInputsAutoLogged();
  private final HoodIO io;
  private final Debouncer calibrationCurrentDebouncer =
      new Debouncer(HoodConstants.kCalibrationDebounceTimeSec, Debouncer.DebounceType.kBoth);

  public Hood(final HoodIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.readInputs(inputs);
    Logger.processInputs("Hood", inputs);
  }

  public Command positionSetpointCommand(DoubleSupplier setpoint) {
    return run(() -> {
          double target = clampToRange(setpoint.getAsDouble());
          setPositionSetpointImpl(target);
        })
        .withName("Hood positionSetpointCommand");
  }

  public Command waitForPosition(DoubleSupplier setpoint, double toleranceRadians) {
    return new WaitUntilCommand(
            () -> {
              double target = clampToRange(setpoint.getAsDouble());
              return Math.abs(inputs.positionRad - target) < toleranceRadians;
            })
        .withName("Hood wait for position");
  }

  public Command resetToLimitCommand() {
    return run(() -> io.setOpenloopVoltage(HoodConstants.kCalibrationVoltage))
        .until(this::isCalibrationStalled)
        .andThen(
            runOnce(
                () -> {
                  io.setOpenloopVoltage(0.0);
                  io.setRotorPosition(HoodConstants.kHoodMinPositionRadians);
                }))
        .withName("Hood Reset to Limit");
  }

  private double clampToRange(double setpoint) {
    return MathUtil.clamp(
        setpoint, HoodConstants.kHoodMinPositionRadians, HoodConstants.kHoodMaxPositionRadians);
  }

  private void setPositionSetpointImpl(double setpoint) {
    Logger.recordOutput("Hood/API/setPositionSetpoint/setpoint", setpoint);
    io.setPositionSetpoint(setpoint);
  }

  private boolean isCalibrationStalled() {
    boolean currentOverThreshold =
        calibrationCurrentDebouncer.calculate(
            Math.abs(inputs.currentStatorAmps) >= HoodConstants.kCalibrationCurrentThreshold);
    return Math.abs(inputs.velocityRadPerSec)
            <= HoodConstants.kCalibrationVelocityThresholdRadPerSec
        && currentOverThreshold;
  }
}
