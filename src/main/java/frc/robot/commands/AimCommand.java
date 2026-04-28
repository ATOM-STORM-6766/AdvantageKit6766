package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.RobotState;
import frc.robot.subsystems.flywheel.FlywheelConstants;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class AimCommand {
  private static final double ANGLE_TOLERANCE_RAD =
      Constants.DriveControlConstants.JoystickAngleHold.TOLERANCE_RAD;

  private static boolean isRobotAtTargetAngle(RobotContainer container) {
    Rotation2d currentYaw = RobotState.getInstance().getRobotPose().getRotation();
    Rotation2d targetYaw = container.getAimSubsystem().getRobotYawRad();
    double error = Math.abs(currentYaw.minus(targetYaw).getRadians());
    return error < ANGLE_TOLERANCE_RAD;
  }

  public static Command prepare(
      RobotContainer container,
      Supplier<Angle> hoodPitchSupplier,
      Supplier<Rotation2d> robotYaw,
      Supplier<AngularVelocity> robotYawRate,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier) {
    return Commands.parallel(
            DriveCommands.joystickDriveAtAngle(
                container.getDrive(),
                () -> xSupplier.getAsDouble() * 0.6,
                () -> ySupplier.getAsDouble() * 0.6,
                robotYaw),
            container.getHood().positionSetpointCommand(hoodPitchSupplier),
            container.getFlywheel().setVelocity(container.getAimSubsystem()::getFlywheelVelocity)
            // Commands.waitUntil(() -> isRobotAtTargetAngle(container))
            //     .andThen(
            //         container
            //             .getFlywheel()
            //             .waitForVelocity(
            //                 container.getAimSubsystem()::getFlywheelVelocity,
            //                 FlywheelConstants.kVelocityToleranceRps))
            //     .andThen(
            //         GamePieceCommands.feedAndStow(container.getIntake(), container.getFeeder(),
            // 0))
            )
        .withName("Prepare Aim");
  }

  public static Command noMove(
      RobotContainer container,
      Supplier<Angle> hoodPitchSupplier,
      Supplier<Rotation2d> robotYaw,
      Supplier<AngularVelocity> robotYawRate) {
    return Commands.parallel(
            container.getHood().positionSetpointCommand(hoodPitchSupplier),
            container.getFlywheel().setVelocity(container.getAimSubsystem()::getFlywheelVelocity),
            Commands.waitUntil(() -> isRobotAtTargetAngle(container))
                .andThen(
                    container
                        .getFlywheel()
                        .waitForVelocity(
                            container.getAimSubsystem()::getFlywheelVelocity,
                            FlywheelConstants.kVelocityToleranceRps))
                .andThen(
                    GamePieceCommands.feedAndStow(container.getIntake(), container.getFeeder(), 0)))
        .withName("Prepare Aim No Move");
  }

  public static Command noMoveShootAtTarget(
      RobotContainer container, Supplier<Translation3d> targetSupplier) {
    return Commands.parallel(
            Commands.startEnd(
                () -> container.getAimSubsystem().setTargetSupplier(targetSupplier),
                container.getAimSubsystem()::clearTargetSupplier,
                container.getAimSubsystem()),
            noMove(
                container,
                container.getAimSubsystem()::getHoodPitch,
                container.getAimSubsystem()::getRobotYawRad,
                container.getAimSubsystem()::getRobotYawRateRadPerSec))
        .withName("No Move Shoot At Target");
  }

  public static Command autoAimAtTarget(
      RobotContainer container,
      Supplier<Translation3d> targetSupplier,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier) {

    return new ParallelCommandGroup(
            Commands.startEnd(
                () -> container.getAimSubsystem().setTargetSupplier(targetSupplier),
                container.getAimSubsystem()::clearTargetSupplier,
                container.getAimSubsystem()),
            prepare(
                container,
                container.getAimSubsystem()::getHoodPitch,
                container.getAimSubsystem()::getRobotYawRad,
                container.getAimSubsystem()::getRobotYawRateRadPerSec,
                xSupplier,
                ySupplier))
        .withName("Auto Aim At Target");
  }
}
