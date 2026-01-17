package frc.robot.subsystems.turret;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.util.RobotState;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class Turret extends SubsystemBase {
  public enum InitState {
    UNINITIALIZED,
    INITIALIZING,
    INITIALIZED
  }

  private final TurretInputsAutoLogged inputs = new TurretInputsAutoLogged();
  private final TurretIO io;
  private final RobotState robotState;
  private final Debouncer calibrationCurrentDebouncer =
      new Debouncer(0.25, Debouncer.DebounceType.kBoth);
  private InitState initState = InitState.UNINITIALIZED;

  public Turret(final TurretIO io, RobotState robotState) {
    this.io = io;
    this.robotState = robotState;
  }

  public void setTeleopDefaultCommand() {}

  @Override
  public void periodic() {
    double timestamp = Timer.getFPGATimestamp();
    io.readInputs(inputs);

    robotState.addTurretRotation(new Rotation2d(inputs.positionRad));

    Logger.processInputs("Turret", inputs);
    Logger.recordOutput("Turret/WorldPose", robotState.getTurretWorldPose());
    Logger.recordOutput("Turret/InitState", initState.toString());
    Logger.recordOutput("Turret/latencyPeriodicSec", Timer.getFPGATimestamp() - timestamp);
  }

  public Command positionSetpointCommand(DoubleSupplier radiansFromCenter, DoubleSupplier ffVel) {
    return run(() -> {
          double setpoint = clampToRange(radiansFromCenter.getAsDouble());
          setPositionSetpointImpl(setpoint, ffVel.getAsDouble());
        })
        .withName("Turret positionSetpointCommand");
  }

  public Command openLoopVoltageCommand(DoubleSupplier voltage) {
    return Commands.startEnd(
            () -> io.setOpenloopVoltage(voltage.getAsDouble()),
            () -> io.setOpenloopVoltage(0.0),
            this)
        .withName("Turret openLoopVoltageCommand");
  }

  public Command waitForPosition(DoubleSupplier radiansFromCenter, double toleranceRadians) {
    return new WaitUntilCommand(
            () -> {
              double target = clampToRange(radiansFromCenter.getAsDouble());
              return Math.abs(inputs.positionRad - target) < toleranceRadians;
            })
        .withName("Turret wait for position");
  }

  public Command resetToLimitCommand() {
    return Commands.runOnce(() -> initState = InitState.INITIALIZING)
        .andThen(
            run(() -> io.setOpenloopVoltage(TurretConstants.kCalibrationVoltage))
                .until(this::isCalibrationStalled)
                .andThen(
                    runOnce(
                        () -> {
                          io.setOpenloopVoltage(0.0);
                          io.setRotorPosition(TurretConstants.kTurretMinPositionRadians);
                          initState = InitState.INITIALIZED;
                        })))
        .withName("Turret Reset to Limit");
  }

  private double clampToRange(double radiansFromCenter) {
    return MathUtil.clamp(
        radiansFromCenter,
        TurretConstants.kTurretMinPositionRadians,
        TurretConstants.kTurretMaxPositionRadians);
  }

  private void setPositionSetpointImpl(double radiansFromCenter, double radsPerSec) {
    Logger.recordOutput("Turret/API/setPositionSetpoint/radiansFromCenter", radiansFromCenter);
    Logger.recordOutput("Turret/API/setPositionSetpoint/radsPerSec", radsPerSec);
    io.setPositionSetpoint(radiansFromCenter, radsPerSec);
  }

  private boolean isCalibrationStalled() {
    boolean currentOverThreshold =
        calibrationCurrentDebouncer.calculate(
            inputs.currentStatorAmps >= TurretConstants.kCalibrationCurrentThreshold);
    return Math.abs(inputs.velocityRadPerSec)
            <= TurretConstants.kCalibrationVelocityThresholdRadPerSec
        && currentOverThreshold;
  }
}
