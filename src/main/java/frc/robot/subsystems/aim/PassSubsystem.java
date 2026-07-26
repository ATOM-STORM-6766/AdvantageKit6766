package frc.robot.subsystems.aim;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.util.GenericShooterResolver;
import frc.robot.util.GenericShooterResolver.ShooterInput;
import frc.robot.util.GenericShooterResolver.ShooterSetpoint;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class PassSubsystem extends SubsystemBase {
  private ShooterSetpoint lastValidSetpoint = createDefaultSetpoint();
  private ShooterSetpoint latestSetpoint = ShooterSetpoint.invalid();
  private Supplier<Translation3d> targetSupplier;

  public void setTargetSupplier(Supplier<Translation3d> targetSupplier) {
    this.targetSupplier = targetSupplier;
  }

  public void clearTargetSupplier() {
    targetSupplier = null;
  }

  @Override
  public void periodic() {
    if (targetSupplier == null) {
      latestSetpoint = ShooterSetpoint.invalid();
      RobotState.getInstance().setAimSetpoint(latestSetpoint);
      return;
    }

    Translation3d target = targetSupplier.get();
    var robotPose = RobotState.getInstance().getRobotPose();
    var robotSpeeds = RobotState.getInstance().getFieldRelativeSpeeds();

    ShooterInput input =
        new ShooterInput(
            robotPose,
            robotSpeeds,
            target,
            lookaheadSeconds -> RobotState.getInstance().getRobotPose(lookaheadSeconds));
    ShooterSetpoint setpoint = GenericShooterResolver.resolve(input, Constants.PASS_CONFIG);

    latestSetpoint = setpoint;
    RobotState.getInstance().setAimSetpoint(setpoint);

    Logger.recordOutput("Pass/SetpointValid", setpoint.isValid);
    Logger.recordOutput("Pass/ActualTarget", new Pose3d(target, new Rotation3d()));

    if (setpoint.isValid) {
      lastValidSetpoint = setpoint;
      Logger.recordOutput("Pass/DesiredRobotYaw", setpoint.robotYaw);
      Logger.recordOutput("Pass/DesiredRobotYawRate", setpoint.robotYawRate);
      Logger.recordOutput("Pass/HoodPitch", setpoint.hoodPitch);
      Logger.recordOutput("Pass/TimeOfFlight", setpoint.timeOfFlightSeconds);
      Logger.recordOutput("Pass/FlywheelRps", setpoint.flywheelRps);
      if (setpoint.virtualTarget != null) {
        Logger.recordOutput(
            "Pass/VirtualTarget", new Pose3d(setpoint.virtualTarget, new Rotation3d()));
      }
    }
  }

  public AngularVelocity getFlywheelVelocity() {
    return lastValidSetpoint.flywheelRps;
  }

  public Angle getHoodPitch() {
    return lastValidSetpoint.hoodPitch;
  }

  public Rotation2d getRobotYawRad() {
    return Rotation2d.fromRadians(lastValidSetpoint.robotYaw.in(Radians));
  }

  public AngularVelocity getRobotYawRateRadPerSec() {
    return lastValidSetpoint.robotYawRate;
  }

  public boolean isSetpointValid() {
    return latestSetpoint.isValid;
  }

  private static ShooterSetpoint createDefaultSetpoint() {
    ShooterSetpoint s = new ShooterSetpoint();
    s.isValid = false;
    s.hoodPitch = Degrees.of(0.0);
    s.robotYaw = Degrees.of(0.0);
    s.robotYawRate = RadiansPerSecond.of(0.0);
    s.flywheelRps = RotationsPerSecond.of(0.0);
    return ShooterSetpoint.invalid();
  }
}
