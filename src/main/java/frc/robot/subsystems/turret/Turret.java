package frc.robot.subsystems.turret;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.util.RobotState;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class Turret extends SubsystemBase {
  private final TurretInputsAutoLogged inputs = new TurretInputsAutoLogged();
  private final TurretIO io;
  private final RobotState robotState;
  private double turretPositionSetpointRadiansFromCenter = 0.0;

  public Turret(final TurretIO io, RobotState robotState) {
    this.io = io;
    this.robotState = robotState;
  }

  public void setTeleopDefaultCommand() {
    this.setDefaultCommand(
        run(() -> {
              setPositionSetpointImpl(turretPositionSetpointRadiansFromCenter, 0.0);
            })
            .withName("Turret Maintain Setpoint (default)"));
  }

  @Override
  public void periodic() {
    double timestamp = Timer.getFPGATimestamp();
    io.readInputs(inputs);
    Logger.processInputs("Turret", inputs);

    robotState.addTurretUpdates(
        timestamp,
        Rotation2d.fromRadians(inputs.positionRad),
        inputs.positionRad,
        inputs.velocityRadPerSec);

    Logger.recordOutput("Turret/WorldPose", robotState.getTurretWorldPose());
    Logger.recordOutput("Turret/latencyPeriodicSec", Timer.getFPGATimestamp() - timestamp);
  }

  public Command positionSetpointCommand(DoubleSupplier radiansFromCenter, DoubleSupplier ffVel) {
    return run(() -> {
          double setpoint = clampToRange(radiansFromCenter.getAsDouble());
          setPositionSetpointImpl(setpoint, ffVel.getAsDouble());
          turretPositionSetpointRadiansFromCenter = setpoint;
        })
        .withName("Turret positionSetpointCommand");
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
    return runOnce(() -> io.startCalibration())
        .andThen(
            run(() -> {})
                .until(() -> inputs.calibrationState == TurretIO.CalibrationState.CALIBRATED))
        .andThen(
            runOnce(
                () -> {
                  turretPositionSetpointRadiansFromCenter =
                      TurretConstants.kTurretMinPositionRadians;
                  Logger.recordOutput("Turret/ResetComplete", true);
                }))
        .withName("Turret Reset to Limit");
  }

  private double clampToRange(double radiansFromCenter) {
    return MathUtil.clamp(
        radiansFromCenter,
        TurretConstants.kTurretMinPositionRadians,
        TurretConstants.kTurretMaxPositionRadians);
  }

  private void setPositionSetpointImpl(double radiansFromCenter, double radPerS) {
    Logger.recordOutput("Turret/API/setPositionSetpoint/radiansFromCenter", radiansFromCenter);
    io.setPositionSetpoint(radiansFromCenter, radPerS);
  }
}
