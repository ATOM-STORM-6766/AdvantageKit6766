package frc.robot.subsystems.flywheel;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.flywheel.FlywheelIO.FlywheelSetpointRps;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class Flywheel extends SubsystemBase {
  private final FlywheelInputsAutoLogged inputs = new FlywheelInputsAutoLogged();
  private final FlywheelIO io;
  private FlywheelSetpointRps velocitySetpoint = new FlywheelSetpointRps(0, 0, 0);

  public Flywheel(final FlywheelIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.readInputs(inputs);
    Logger.processInputs("Flywheel", inputs);
  }

  public Command velocitySetpointCommand(Supplier<FlywheelSetpointRps> setpointSupplier) {
    return run(() -> {
          FlywheelSetpointRps setpoint = setpointSupplier.get();
          setVelocityImpl(setpoint);
          velocitySetpoint = setpoint;
        })
        .withName("Flywheel Velocity Control");
  }

  public Command waitForVelocity(
      Supplier<FlywheelSetpointRps> setpointSupplier, double toleranceRps) {
    return new WaitUntilCommand(
            () -> {
              FlywheelSetpointRps target = setpointSupplier.get();
              boolean at0 = Math.abs(inputs.velocityRps0 - target.rps0()) < toleranceRps;
              boolean at1 = Math.abs(inputs.velocityRps1 - target.rps1()) < toleranceRps;
              boolean at2 = Math.abs(inputs.velocityRps2 - target.rps2()) < toleranceRps;
              return at0 && at1 && at2;
            })
        .withName("Flywheel Wait For Velocity");
  }

  public boolean isAtTargetVelocity() {
    double tol = FlywheelConstants.kFlywheelVelocityToleranceRps;
    return Math.abs(velocitySetpoint.rps0() - inputs.velocityRps0) < tol
        && Math.abs(velocitySetpoint.rps1() - inputs.velocityRps1) < tol
        && Math.abs(velocitySetpoint.rps2() - inputs.velocityRps2) < tol;
  }

  private void setVelocityImpl(FlywheelSetpointRps setpoint) {
    Logger.recordOutput("Flywheel/API/setVelocity/rps0", setpoint.rps0());
    Logger.recordOutput("Flywheel/API/setVelocity/rps1", setpoint.rps1());
    Logger.recordOutput("Flywheel/API/setVelocity/rps2", setpoint.rps2());
    io.setVelocity(setpoint.rps0(), setpoint.rps1(), setpoint.rps2());
  }
}
