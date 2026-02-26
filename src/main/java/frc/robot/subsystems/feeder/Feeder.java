package frc.robot.subsystems.feeder;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class Feeder extends SubsystemBase {
  private final FeederInputsAutoLogged inputs = new FeederInputsAutoLogged();
  private final FeederIO io;

  public Feeder(final FeederIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.readInputs(inputs);
    Logger.processInputs("Feeder", inputs);
  }

  public Command setFeederVelocityCommand(
      Supplier<AngularVelocity> intakeSupplier, Supplier<AngularVelocity> shooterSupplier) {
    return Commands.runOnce(
            () -> {
              setIntakeVelocityImpl(intakeSupplier.get());
              setShooterVelocityImpl(shooterSupplier.get());
            })
        .withName("Feeder Feeder Velocity Control");
  }

  public Command stopCommand() {
    return runOnce(io::stop).withName("Feeder Stop");
  }

  private void setShooterVelocityImpl(AngularVelocity velocity) {
    Logger.recordOutput("Feeder/API/setShooterVelocity", velocity);
    io.setShooterVelocity(velocity);
  }

  private void setIntakeVelocityImpl(AngularVelocity velocity) {
    Logger.recordOutput("Feeder/API/setIntakeVelocity", velocity);
    io.setIntakeVelocity(velocity);
  }
}
