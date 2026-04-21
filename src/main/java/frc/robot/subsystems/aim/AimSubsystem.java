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
import frc.robot.util.GenericShooterResolver.ShooterSetpoint;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class AimSubsystem extends SubsystemBase {
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

    ShooterSetpoint setpoint =
        GenericShooterResolver.resolve(robotPose, robotSpeeds, target, Constants.SHOOTER_CONFIG);

    latestSetpoint = setpoint;
    RobotState.getInstance().setAimSetpoint(setpoint);

    Logger.recordOutput("Aiming/SetpointValid", setpoint.isValid);
    Logger.recordOutput("Aiming/ActualTarget", new Pose3d(target, new Rotation3d()));

    if (setpoint.isValid) {
      lastValidSetpoint = setpoint;
      Logger.recordOutput("Aiming/DesiredRobotYaw", setpoint.robotYaw);
      Logger.recordOutput("Aiming/DesiredRobotYawRate", setpoint.robotYawRate);
      Logger.recordOutput("Aiming/HoodPitch", setpoint.hoodPitch);
      Logger.recordOutput("Aiming/TimeOfFlight", setpoint.timeOfFlightSeconds);
      Logger.recordOutput("Aiming/FlywheelRps", setpoint.flywheelRps);
      if (setpoint.virtualTarget != null) {
        Logger.recordOutput(
            "Aiming/VirtualTarget", new Pose3d(setpoint.virtualTarget, new Rotation3d()));
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
