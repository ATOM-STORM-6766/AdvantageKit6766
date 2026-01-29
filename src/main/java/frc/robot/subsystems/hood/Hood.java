package frc.robot.subsystems.hood;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotState;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class Hood extends SubsystemBase {
  public enum InitState {
    UNINITIALIZED,
    INITIALIZING,
    INITIALIZED
  }

  private final HoodInputsAutoLogged inputs = new HoodInputsAutoLogged();
  private final RobotState robotState = RobotState.getInstance();
  private final HoodIO io;
  private final Debouncer calibrationCurrentDebouncer =
      new Debouncer(0.25, Debouncer.DebounceType.kBoth);
  private InitState initState = InitState.UNINITIALIZED;

  public Hood(final HoodIO io) {
    this.io = io;
  }

  public void setTeleopDefaultCommand() {}

  @Override
  public void periodic() {
    double timestamp = Timer.getFPGATimestamp();
    io.readInputs(inputs);

    robotState.addHoodRotation(new Rotation2d(inputs.positionRad));

    Logger.processInputs("Hood", inputs);
    Logger.recordOutput("Hood/WorldPose", robotState.getHoodWorldPose());
    Logger.recordOutput("Hood/InitState", initState.toString());
    Logger.recordOutput("Hood/latencyPeriodicSec", Timer.getFPGATimestamp() - timestamp);
  }

  public Command positionSetpointCommand(
      DoubleSupplier radiansFromCenter, DoubleSupplier radsPerSec) {
    return run(() -> {
          double setpoint = clampToRange(radiansFromCenter.getAsDouble());
          setPositionSetpointImpl(setpoint, radsPerSec.getAsDouble());
        })
        .withName("Hood positionSetpointCommand");
  }

  public Command openLoopVoltageCommand(DoubleSupplier voltage) {
    return Commands.startEnd(
            () -> io.setOpenloopVoltage(voltage.getAsDouble()),
            () -> io.setOpenloopVoltage(0.0),
            this)
        .withName("Hood openLoopVoltageCommand");
  }

  public Command waitForPosition(DoubleSupplier radiansFromCenter, double toleranceRadians) {
    return new WaitUntilCommand(
            () -> {
              double target = clampToRange(radiansFromCenter.getAsDouble());
              return Math.abs(inputs.positionRad - target) < toleranceRadians;
            })
        .withName("Hood wait for position");
  }

  public Command resetToLimitCommand() {
    return Commands.runOnce(() -> initState = InitState.INITIALIZING)
        .andThen(
            run(() -> io.setOpenloopVoltage(HoodConstants.kCalibrationVoltage))
                .until(this::isCalibrationStalled)
                .andThen(
                    runOnce(
                        () -> {
                          io.setOpenloopVoltage(0.0);
                          io.setRotorPosition(HoodConstants.kHoodMinPositionRadians);
                          initState = InitState.INITIALIZED;
                        })))
        .withName("Hood Reset to Limit");
  }

  private double clampToRange(double radiansFromCenter) {
    return MathUtil.clamp(
        radiansFromCenter,
        HoodConstants.kHoodMinPositionRadians,
        HoodConstants.kHoodMaxPositionRadians);
  }

  private void setPositionSetpointImpl(double radiansFromCenter, double radsPerSec) {
    Logger.recordOutput("Hood/API/setPositionSetpoint/radiansFromCenter", radiansFromCenter);
    Logger.recordOutput("Hood/API/setPositionSetpoint/radsPerSec", radsPerSec);
    io.setPositionSetpoint(radiansFromCenter, radsPerSec);
  }

  private boolean isCalibrationStalled() {
    boolean currentOverThreshold =
        calibrationCurrentDebouncer.calculate(
            inputs.currentStatorAmps >= HoodConstants.kCalibrationCurrentThreshold);
    return Math.abs(inputs.velocityRadPerSec)
            <= HoodConstants.kCalibrationVelocityThresholdRadPerSec
        && currentOverThreshold;
  }
}
