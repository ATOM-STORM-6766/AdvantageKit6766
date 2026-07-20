package frc.robot.subsystems.flywheel;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class Flywheel extends SubsystemBase {
  private final FlywheelInputsAutoLogged inputs = new FlywheelInputsAutoLogged();
  private final FlywheelIO io;
  private AngularVelocity targetVelocity = RotationsPerSecond.of(0);

  private final Debouncer feedBoostDebouncer = new Debouncer(0.06, Debouncer.DebounceType.kFalling);

  public Flywheel() {
    this.io = FlywheelIO.getIO();
  }

  @Override
  public void periodic() {
    io.readInputs(inputs);
    Logger.processInputs("Flywheel", inputs);
  }

  public Command setVelocity(Supplier<AngularVelocity> velocitySupplier) {
    return run(() -> {
          AngularVelocity velocity =
              velocitySupplier
                  .get()
                  .plus(RotationsPerSecond.of(feedBoostDebouncer.calculate(false) ? 0.5 : 0.0));
          setVelocityImpl(velocity);
        })
        .withName("Flywheel Velocity Control");
  }

  public Command stopCommand() {
    return runOnce(() -> io.stop()).withName("Flywheel Stop");
  }

  /** 触发飞轮送球速度提升（+0.5 rps，持续 0.06s） */
  public void triggerFeedBoost() {
    feedBoostDebouncer.calculate(true);
  }

  public Command waitForVelocity(Supplier<AngularVelocity> velocitySupplier, double toleranceRps) {
    return new WaitUntilCommand(
            () -> {
              double targetRps = Math.abs(velocitySupplier.get().in(RotationsPerSecond));
              boolean at0 =
                  Math.abs(inputs.velocity0.in(RotationsPerSecond) - targetRps) < toleranceRps;
              boolean at1 =
                  Math.abs(inputs.velocity1.in(RotationsPerSecond) - targetRps) < toleranceRps;
              boolean at2 =
                  Math.abs(inputs.velocity2.in(RotationsPerSecond) - targetRps) < toleranceRps;
              boolean at3 =
                  Math.abs(inputs.velocity3.in(RotationsPerSecond) - targetRps) < toleranceRps;

              Logger.recordOutput("Flywheel/API/waitForVelocity", at0 && at1 && at2 && at3);

              return at0 && at1 && at2 && at3;
            })
        .withName("Flywheel Wait For Velocity");
  }

  public boolean isAtTargetVelocity() {
    double targetRps = targetVelocity.in(RotationsPerSecond);
    if (targetRps == 0) return false;
    double tol = FlywheelConstants.kVelocityToleranceRps;
    return Math.abs(inputs.velocity0.in(RotationsPerSecond) - targetRps) < tol
        && Math.abs(inputs.velocity1.in(RotationsPerSecond) - targetRps) < tol
        && Math.abs(inputs.velocity2.in(RotationsPerSecond) - targetRps) < tol
        && Math.abs(inputs.velocity3.in(RotationsPerSecond) - targetRps) < tol;
  }

  private void setVelocityImpl(AngularVelocity velocity) {
    targetVelocity = velocity;
    Logger.recordOutput("Flywheel/API/setVelocity", velocity);
    io.setFlywheelVelocity(velocity);
  }
}
