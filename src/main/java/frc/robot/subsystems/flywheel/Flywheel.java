package frc.robot.subsystems.flywheel;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotState;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class Flywheel extends SubsystemBase {
  private final FlywheelInputsAutoLogged inputs = new FlywheelInputsAutoLogged();
  private final FlywheelIO io;
  private final RobotState robotState = RobotState.getInstance();
  private double velocitySetpointRPM = 0.0;

  public Flywheel(final FlywheelIO io) {
    this.io = io;
  }

  public void setTeleopDefaultCommand() {}

  @Override
  public void periodic() {
    double timestamp = Timer.getFPGATimestamp();
    io.readInputs(inputs);
    Logger.processInputs("Flywheel", inputs);

    Logger.recordOutput("Flywheel/velocitySetpointRPM", velocitySetpointRPM);
    Logger.recordOutput("Flywheel/velocityErrorRPM", velocitySetpointRPM - inputs.velocityRPM);
    Logger.recordOutput("Flywheel/atTarget", isAtTargetVelocity());
    Logger.recordOutput("Flywheel/WorldPose", robotState.getFlywheelWorldPose());
    Logger.recordOutput("Flywheel/latencyPeriodicSec", Timer.getFPGATimestamp() - timestamp);
  }

  public Command velocitySetpointCommand(DoubleSupplier velocityRPM) {
    return run(() -> {
          double setpoint = velocityRPM.getAsDouble();
          setVelocityImpl(setpoint, 0.0);
          velocitySetpointRPM = setpoint;
        })
        .withName("Flywheel Velocity Control");
  }

  public Command waitForVelocity(DoubleSupplier velocityRPM, double toleranceRPM) {
    return new WaitUntilCommand(
            () -> {
              double error = Math.abs(inputs.velocityRPM - velocityRPM.getAsDouble());
              return error < toleranceRPM;
            })
        .withName("Flywheel Wait For Velocity");
  }

  public boolean isAtTargetVelocity() {
    return Math.abs(velocitySetpointRPM - inputs.velocityRPM)
        < FlywheelConstants.kFlywheelVelocityTolerance;
  }

  private void setVelocityImpl(double velocityRPM, double ffVolts) {
    Logger.recordOutput("Flywheel/API/setVelocity/velocityRPM", velocityRPM);
    Logger.recordOutput("Flywheel/API/setVelocity/ffVolts", ffVolts);
    io.setVelocity(velocityRPM, ffVolts);
  }
}
