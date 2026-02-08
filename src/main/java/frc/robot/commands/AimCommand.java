package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.RobotState;
import frc.robot.util.GenericShooterResolver;
import frc.robot.util.GenericShooterResolver.ShooterSetpoint;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class AimCommand {

  public static Command resetAllToLimit(RobotContainer container) {
    return new ParallelCommandGroup(
            container.getHood().resetToLimitCommand(), container.getIntake().resetToLimitCommand())
        .withName("Reset To Limit");
  }

  public static Command aimAtTarget(
      RobotContainer container,
      DoubleSupplier hoodPitchRadSupplier,
      DoubleSupplier robotYawRadSupplier,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier) {
    return new ParallelCommandGroup(
            DriveCommands.joystickDriveAtAngle(
                container.getDrive(),
                xSupplier,
                ySupplier,
                () -> Rotation2d.fromRadians(robotYawRadSupplier.getAsDouble())),
            container.getHood().positionSetpointCommand(hoodPitchRadSupplier))
        .withName("Aim At Target");
  }

  public static Command autoAimAtTarget(
      RobotContainer container,
      Supplier<Translation3d> targetSupplier,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier) {
    final AtomicReference<ShooterSetpoint> lastValidSetpoint =
        new AtomicReference<>(createDefaultSetpoint());
    return new ParallelCommandGroup(
            new RunCommand(() -> updateSetpoint(container, targetSupplier, lastValidSetpoint)),
            aimAtTarget(
                container,
                () -> lastValidSetpoint.get().hoodPitch,
                () -> lastValidSetpoint.get().robotYaw,
                xSupplier,
                ySupplier))
        .withName("Auto Aim At Target");
  }

  private static ShooterSetpoint createDefaultSetpoint() {
    ShooterSetpoint s = new ShooterSetpoint();
    s.robotYaw = Math.toRadians(180.0);
    s.hoodPitch = Math.toRadians(30.0);
    s.isValid = false;
    return s;
  }

  private static void updateSetpoint(
      RobotContainer container,
      Supplier<Translation3d> targetSupplier,
      AtomicReference<ShooterSetpoint> lastValidSetpoint) {

    var robotPose = RobotState.getInstance().getRobotPose();
    var robotSpeeds = RobotState.getInstance().getFieldRelativeSpeeds();
    var target = targetSupplier.get();

    ShooterSetpoint setpoint =
        GenericShooterResolver.resolve(robotPose, robotSpeeds, target, Constants.SHOOTER_CONFIG);

    Logger.recordOutput("Aiming/SetpointValid", setpoint.isValid);
    Logger.recordOutput("Aiming/ActualTarget", new Pose3d(target, new Rotation3d()));

    if (setpoint.isValid) {
      lastValidSetpoint.set(setpoint);

      // Log debug info
      Logger.recordOutput("Aiming/DesiredRobotYawDeg", Math.toDegrees(setpoint.robotYaw));
      Logger.recordOutput("Aiming/HoodPitchDeg", Math.toDegrees(setpoint.hoodPitch));
      Logger.recordOutput("Aiming/TimeOfFlight", setpoint.timeOfFlightSeconds);
      Logger.recordOutput("Aiming/FlywheelRps", setpoint.flywheelRps);
      if (setpoint.virtualTarget != null) {
        Logger.recordOutput(
            "Aiming/VirtualTarget", new Pose3d(setpoint.virtualTarget, new Rotation3d()));
      }
    }
  }
}
