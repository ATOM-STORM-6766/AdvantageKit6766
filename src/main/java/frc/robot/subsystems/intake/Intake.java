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
    STOWED(IntakeConstants.kIntakeMinPosition),
    COLLISION(Rotations.of(2.1));

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
  private final Debouncer slipCurrentDebouncer =
      new Debouncer(IntakeConstants.kSlipCalibrationSec, Debouncer.DebounceType.kBoth);

  private final Debouncer collisionDebouncer =
      new Debouncer(IntakeConstants.kCollisionDebounceTimeSec, Debouncer.DebounceType.kBoth);
  private final Debouncer collisionCooldownDebouncer =
      new Debouncer(IntakeConstants.kCollisionCooldownSec, Debouncer.DebounceType.kFalling);
  private boolean collisionDetected = false;
  private Position lastTargetPosition = Position.STOWED;

  public Intake() {
    this.io = IntakeIO.getIO();
    setDefaultCommand(defaultPositionCommand());
  }

  @Override
  public void periodic() {
    io.readInputs(inputs);
    Logger.processInputs("Intake", inputs);

    boolean rawCollision = checkCollisionProtection();
    Logger.recordOutput("Intake/RawCollision", rawCollision);

    collisionDetected = collisionCooldownDebouncer.calculate(rawCollision);
    Logger.recordOutput("Intake/CollisionCooldown", collisionDetected);
  }

  /** 检测碰撞，返回是否触发了碰撞保护 */
  private boolean checkCollisionProtection() {
    // // 仅在初始化完成后启用
    // if (state != State.INITIALIZED) return false;

    // if (inputs.intakePositionRotation.lte(Position.COLLISION.getSetpoint())) {
    //   return false;
    // }

    // boolean velocityNegative =
    //     inputs.positionVelocity.in(RadiansPerSecond) <=
    // IntakeConstants.kCollisionVelocityThreshold;

    // boolean torquePositiveHigh =
    //     inputs.positionTorqueAmps.in(Amps) >= IntakeConstants.kCollisionCurrentThreshold;

    // boolean isCollision = collisionDebouncer.calculate(velocityNegative && torquePositiveHigh);

    // if (isCollision) {
    //   return true;
    // }

    return false;
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

  public Command unProtectedExtend() {
    return run(() -> io.setPositionVoltage(Volts.of(3.0)))
        .until(this::isSlipStalled)
        .andThen(
            () -> {
              io.setPositionVoltage(Volts.of(0));
              io.setIntakeSensorPosition(IntakeConstants.kIntakeMaxPosition);
            });
  }

  public Command stopPosition() {
    return runOnce(() -> io.setPositionVoltage(Volts.of(0.0)));
  }

  public Command setIntakeVelocityCommand(Supplier<Voltage> voltageSupplier) {
    return runOnce(() -> io.setIntakeVelocity(voltageSupplier.get()))
        .withName("Intake Set Intake Velocity");
  }

  public Command setPosCommand(Position targetPosition, boolean once) {
    Runnable action =
        () -> {
          lastTargetPosition = targetPosition;
          if (collisionDetected) {
            setIntakePositionCollisionImpl(Position.COLLISION.getSetpoint());
          } else {
            setIntakePositionWithVelocityImpl(
                targetPosition.getSetpoint(), IntakeConstants.kOutVelocityRPS);
          }
        };
    return once
        ? Commands.runOnce(action)
            .until(() -> isAtPosition(targetPosition.getSetpoint()))
            .withName("Intake Set Position Once")
        : Commands.run(action)
            .until(() -> isAtPosition(targetPosition.getSetpoint()))
            .withName("Intake Set Position Repeat");
  }

  private Command defaultPositionCommand() {
    return run(() -> {
          if (collisionDetected) {
            setIntakePositionCollisionImpl(Position.COLLISION.getSetpoint());
          } else {
            setIntakePositionImpl(lastTargetPosition.getSetpoint());
          }
        })
        .withName("Intake Default Position");
  }

  public Command stopCommand() {
    return runOnce(() -> io.stop()).withName("Intake Stop");
  }

  public Command setSlowStowCommand() {
    Angle target = Position.STOWED.getSetpoint();
    return Commands.run(
            () -> {
              lastTargetPosition = Position.STOWED;
              setIntakePositionWithVelocityImpl(target, IntakeConstants.kSlowStowVelocityRPS);
            })
        .until(() -> isAtPosition(target))
        .andThen(
            Commands.runOnce(
                () -> {
                  io.setIntakeSensorPosition(IntakeConstants.kIntakeMinPosition);
                  setIntakePositionImpl(target);
                }))
        .withName("Intake Slow Stow");
  }

  private void setIntakePositionImpl(Angle targetPositionRotation) {
    Logger.recordOutput("Intake/API/rotationSetpoint", targetPositionRotation);
    io.setIntakePosition(targetPositionRotation);
  }

  private void setIntakePositionCollisionImpl(Angle targetPositionRotation) {
    Logger.recordOutput("Intake/API/rotationSetpoint", targetPositionRotation);
    io.setIntakeCollisionPosition(targetPositionRotation);
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

  private boolean isSlipStalled() {
    boolean currentOverThreshold =
        slipCurrentDebouncer.calculate(
            inputs.positionStatorAmps.abs(Amps) >= IntakeConstants.kCalibrationCurrentThreshold);
    return currentOverThreshold;
  }
}
