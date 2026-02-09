package frc.robot.subsystems.flywheel;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.flywheel.FlywheelIO.FlywheelSetpoint;
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

  public Command flywheelSetpointCommand(Supplier<FlywheelSetpoint> setpointSupplier) {
    return run(() -> {
          FlywheelSetpoint setpoint = setpointSupplier.get();
          setVelocityImpl(setpoint);
        })
        .withName("Flywheel Velocity Control");
  }

  public Command feedVelocity(Supplier<AngularVelocity> setpoint) {
    return run(() -> {
          Logger.recordOutput("Flywheel/API/setVelocity/feeder", setpoint.get());
          io.setFeederVelocity(setpoint.get());
        })
        .withName("Flywheel Feeder Velocity Control");
  }

  public Command setVelocity(
      Supplier<FlywheelSetpoint> setpointSupplier, Supplier<AngularVelocity> feedSetpoint) {
    return run(() -> {
          FlywheelSetpoint setpoint = setpointSupplier.get();
          Logger.recordOutput("Flywheel/API/setVelocity/feeder", feedSetpoint.get());
          io.setFeederVelocity(feedSetpoint.get());
          setVelocityImpl(setpoint);
        })
        .withName("Flywheel Velocity Control");
  }

  public Command stopCommand() {
    return run(() -> io.stop()).withName("Flywheel Stop");
  }

  public Command waitForVelocity(Supplier<FlywheelSetpoint> setpointSupplier, double toleranceRps) {
    return new WaitUntilCommand(
            () -> {
              FlywheelSetpoint target = setpointSupplier.get();
              boolean at0 =
                  Math.abs(inputs.velocity0.minus(target.motor0()).in(RotationsPerSecond))
                      < toleranceRps;
              boolean at1 =
                  Math.abs(inputs.velocity1.minus(target.motor1()).in(RotationsPerSecond))
                      < toleranceRps;
              boolean at2 =
                  Math.abs(inputs.velocity2.minus(target.motor2()).in(RotationsPerSecond))
                      < toleranceRps;
              return at0 && at1 && at2;
            })
        .withName("Flywheel Wait For Velocity");
  }

  private void setVelocityImpl(FlywheelSetpoint setpoint) {
    Logger.recordOutput("Flywheel/API/setVelocity/rps0", setpoint.motor0());
    Logger.recordOutput("Flywheel/API/setVelocity/rps1", setpoint.motor1());
    Logger.recordOutput("Flywheel/API/setVelocity/rps2", setpoint.motor2());
    io.setFlywheelVelocity(setpoint);
  }
}
