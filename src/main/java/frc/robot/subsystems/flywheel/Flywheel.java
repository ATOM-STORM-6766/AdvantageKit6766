package frc.robot.subsystems.flywheel;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class Flywheel extends SubsystemBase {
  private final FlywheelInputsAutoLogged inputs = new FlywheelInputsAutoLogged();
  private final FlywheelIO io;

  public Flywheel(final FlywheelIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Flywheel", inputs);
  }

  public Command setVelocity(Supplier<AngularVelocity> velocitySupplier) {
    return run(() -> {
          AngularVelocity velocity = velocitySupplier.get();
          setVelocityImpl(velocity);
        })
        .withName("Flywheel Velocity Control");
  }

  public Command stopCommand() {
    return runOnce(() -> io.stop()).withName("Flywheel Stop");
  }

  public Command waitForVelocity(Supplier<AngularVelocity> velocitySupplier, double toleranceRps) {
    return new WaitUntilCommand(
            () -> {
              AngularVelocity target = velocitySupplier.get();
              boolean at0 =
                  Math.abs(inputs.velocity0.minus(target).in(RotationsPerSecond)) < toleranceRps;
              boolean at1 =
                  Math.abs(inputs.velocity1.minus(target).in(RotationsPerSecond)) < toleranceRps;
              boolean at2 =
                  Math.abs(inputs.velocity2.minus(target).in(RotationsPerSecond)) < toleranceRps;
              boolean at3 =
                  Math.abs(inputs.velocity3.minus(target).in(RotationsPerSecond)) < toleranceRps;
              return at0 && at1 && at2 && at3;
            })
        .withName("Flywheel Wait For Velocity");
  }

  private void setVelocityImpl(AngularVelocity velocity) {
    Logger.recordOutput("Flywheel/API/setVelocity", velocity);
    io.setFlywheelVelocity(velocity);
  }
}
