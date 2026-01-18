package frc.robot.subsystems.turret;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
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

  public enum CalibrationMethod {
    ABSOLUTE_ENCODER,
    CURRENT_STALL,
    NO_CALIBRATION
  }

  private final TurretInputsAutoLogged inputs = new TurretInputsAutoLogged();
  private final TurretIO io;
  private final RobotState robotState;
  private final Debouncer calibrationCurrentDebouncer =
      new Debouncer(0.25, Debouncer.DebounceType.kBoth);
  private InitState initState = InitState.UNINITIALIZED;
  private CalibrationMethod calibrationMethod = CalibrationMethod.NO_CALIBRATION;

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

    var resolvedAngle =
        solveAbsoluteAngle(inputs.absoluteEncoderPosition, inputs.absoluteEncoder2Position);
    if (resolvedAngle != null) {
      Logger.recordOutput("Turret/resolvedAngle", Units.radiansToDegrees(resolvedAngle));
    } else {
      Logger.recordOutput("Turret/resolvedAngle", Double.NaN);
    }
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
    if (calibrationMethod == CalibrationMethod.ABSOLUTE_ENCODER) {
      return Commands.runOnce(
              () -> {
                initState = InitState.INITIALIZING;

                // Use CRT algorithm to solve absolute angle from dual encoders
                Double solvedAngle =
                    solveAbsoluteAngle(
                        inputs.absoluteEncoderPosition, inputs.absoluteEncoder2Position);

                if (solvedAngle != null) {
                  double clampedPosition = clampToRange(solvedAngle);
                  io.setRotorPosition(clampedPosition);
                  Logger.recordOutput("Turret/Init/solvedAngleRad", solvedAngle);
                  Logger.recordOutput(
                      "Turret/Init/solvedAngleDeg", Units.radiansToDegrees(solvedAngle));
                  Logger.recordOutput("Turret/Init/clampedPositionRad", clampedPosition);
                  Logger.recordOutput("Turret/Init/calibrationSuccess", true);
                  initState = InitState.INITIALIZED;
                }
              })
          .withName("Turret Reset to Limit");
    }

    if (calibrationMethod == CalibrationMethod.CURRENT_STALL) {
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

    if (calibrationMethod == CalibrationMethod.NO_CALIBRATION) {
      return Commands.runOnce(() -> initState = InitState.INITIALIZING)
          .andThen(
              runOnce(
                  () -> {
                    io.setRotorPosition(0.0);
                    initState = InitState.INITIALIZED;
                  }))
          .withName("Turret Reset to Limit");
    }

    return Commands.none();
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

  /**
   * Solve turret absolute angle using dual encoder CRT algorithm.
   *
   * <p>Two encoders with different gear ratios (100/11 and 100/10) produce signals with different
   * periods. Since gcd(10, 11) = 1, we can uniquely determine the turret position within 396
   * degrees (> 360 degrees full rotation).
   *
   * @param encoder1Pos encoder 1 position (0-1 rotations)
   * @param encoder2Pos encoder 2 position (0-1 rotations)
   * @return turret absolute angle in radians, or null if no valid solution found
   */
  private Double solveAbsoluteAngle(double encoder1Pos, double encoder2Pos) {
    double k1 = TurretConstants.kTurretAbsoluteEncoderToTurretRatio; // ~9.09
    double k2 = TurretConstants.kTurretAbsoluteEncoder2ToTurretRatio; // 10.0
    double tolerance = TurretConstants.kCRTToleranceRadians;

    // Convert encoder positions (0-1 rotations) to angles (radians)
    double theta1 = encoder1Pos * 2 * Math.PI; // Encoder 1 angle
    double theta2 = encoder2Pos * 2 * Math.PI; // Encoder 2 angle

    // Generate candidate solutions based on encoder 1
    int numCycles = (int) Math.ceil(k1);
    Double bestSolution = null;
    double minError = Double.MAX_VALUE;

    for (int n = 0; n <= numCycles; n++) {
      double totalAngle1 = theta1 + n * 2 * Math.PI;
      double candidateTurretAngle = totalAngle1 / k1;

      if (candidateTurretAngle >= 2 * Math.PI) {
        break;
      }

      // Verify: calculate expected encoder 2 reading based on candidate angle
      double expectedTheta2 = (candidateTurretAngle * k2) % (2 * Math.PI);
      double diff = Math.abs(expectedTheta2 - theta2);

      // Handle 0/360 degree wraparound
      if (diff > Math.PI) {
        diff = 2 * Math.PI - diff;
      }

      if (diff < tolerance && diff < minError) {
        minError = diff;
        bestSolution = candidateTurretAngle;
      }
    }

    return bestSolution;
  }
}
