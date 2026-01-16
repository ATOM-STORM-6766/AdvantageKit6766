package frc.robot.subsystems.hood;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.util.RobotState;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class Hood extends SubsystemBase {
  private final HoodInputsAutoLogged inputs = new HoodInputsAutoLogged();
  private final RobotState robotState;
  private final HoodIO io;
  private double hoodSetpointRadiansFromCenter = 0.0;

  public Hood(final HoodIO io, RobotState robotState) {
    this.io = io;
    this.robotState = robotState;
  }

  public void setTeleopDefaultCommand() {
    this.setDefaultCommand(
        run(() -> {
              setPositionSetpointImpl(hoodSetpointRadiansFromCenter, 0.0);
            })
            .withName("Hood Maintain Setpoint (default)"));
  }

  @Override
  public void periodic() {
    double timestamp = Timer.getFPGATimestamp();
    io.readInputs(inputs);
    Logger.processInputs("Hood", inputs);
    io.update(inputs);

    robotState.addHoodRotation(new Rotation2d(inputs.positionRad));
    Logger.recordOutput("Hood/WorldPose", robotState.getHoodWorldPose());
    Logger.recordOutput("Hood/latencyPeriodicSec", Timer.getFPGATimestamp() - timestamp);
  }

  public Command positionSetpointCommand(
      DoubleSupplier radiansFromCenter, DoubleSupplier radsPerSec) {
    return run(() -> {
          double setpoint = clampToRange(radiansFromCenter.getAsDouble());
          setPositionSetpointImpl(setpoint, radsPerSec.getAsDouble());
          hoodSetpointRadiansFromCenter = setpoint;
        })
        .withName("Hood positionSetpointCommand");
  }

  public Command waitForPosition(DoubleSupplier radiansFromCenter, double toleranceRadians) {
    return new WaitUntilCommand(
            () -> {
              double target = clampToRange(radiansFromCenter.getAsDouble());
              return Math.abs(inputs.positionRad - target) < toleranceRadians;
            })
        .withName("Hood wait for position");
  }

  private double clampToRange(double radiansFromCenter) {
    return MathUtil.clamp(
        radiansFromCenter,
        HoodConstants.kHoodMinPositionRadians,
        HoodConstants.kHoodMaxPositionRadians);
  }

  private void setPositionSetpointImpl(double radiansFromCenter, double radsPerSec) {
    Logger.recordOutput("Hood/API/setPositionSetpoint/radiansFromCenter", radiansFromCenter);
    Logger.recordOutput("Hood/API/setPositionSetpoint/radsPerSec", radsPerSec);
    io.setPositionSetpoint(radiansFromCenter, radsPerSec);
  }
}
