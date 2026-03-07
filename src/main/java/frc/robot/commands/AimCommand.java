package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class AimCommand {
  private static final LoggedTunableNumber hoodPos = new LoggedTunableNumber("Hood/Position", 30.0);
  private static final LoggedTunableNumber flywheelRPS =
      new LoggedTunableNumber("Flywheel/RPS", 0.0);

  public static Command prepare(
      RobotContainer container,
      Supplier<Angle> hoodPitchSupplier,
      Supplier<Rotation2d> robotYaw,
      Supplier<AngularVelocity> robotYawRate,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier) {
    return Commands.parallel(
            // 启动底盘旋转
            DriveCommands.joystickDriveAtAngle(
                container.getDrive(),
                () -> xSupplier.getAsDouble() * 0.6,
                () -> ySupplier.getAsDouble() * 0.6,
                robotYaw),

            // 启动 Hood 旋转
            container.getHood().positionSetpointCommand(hoodPitchSupplier),

            // 先反转喂料 0.6s，清理卡球
            Commands.sequence(
                // container
                //     .getFlywheel()
                //     .setVelocity(
                //         () ->
                //             new FlywheelSetpoint(
                //                 RotationsPerSecond.of(-30.0),
                //                 RotationsPerSecond.of(-30.0),
                //                 RotationsPerSecond.of(-30.0)))
                //     .andThen(container.getFlywheel().stopCommand())
                //     .withTimeout(0.6),

                // 设置飞轮速度
                container
                    .getFlywheel()
                    .setVelocity(container.getAimSubsystem()::getFlywheelSetpoint)))
        // .setVelocity(
        //     () ->
        //         new FlywheelSetpoint(
        //             RotationsPerSecond.of(flywheelRPS.get()),
        //             RotationsPerSecond.of(flywheelRPS.get()),
        //             RotationsPerSecond.of(flywheelRPS.get())))))
        .withName("Prepare Aim");
  }

  public static Command noMove(
      RobotContainer container,
      Supplier<Angle> hoodPitchSupplier,
      Supplier<Rotation2d> robotYaw,
      Supplier<AngularVelocity> robotYawRate) {
    return Commands.parallel(
            // 启动 Hood 旋转
            container.getHood().positionSetpointCommand(hoodPitchSupplier),

            // 设置飞轮速度
            container.getFlywheel().setVelocity(container.getAimSubsystem()::getFlywheelSetpoint))
        .withName("Prepare Aim No Move");
  }

  public static Command noMoveShootAtTarget(
      RobotContainer container, Supplier<Translation3d> targetSupplier) {
    return Commands.parallel(
            // 设置自动目标（冲突时会被清除）
            Commands.startEnd(
                () -> container.getAimSubsystem().setTargetSupplier(targetSupplier),
                container.getAimSubsystem()::clearTargetSupplier,
                container.getAimSubsystem()),

            // 启动 Hood 旋转和飞轮速度
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
            // 设置自动目标（冲突时会被清除）
            Commands.startEnd(
                () -> container.getAimSubsystem().setTargetSupplier(targetSupplier),
                container.getAimSubsystem()::clearTargetSupplier,
                container.getAimSubsystem()),

            // 准备自动瞄准时的底盘和 Hood 旋转和飞轮速度
            prepare(
                container,
                container.getAimSubsystem()::getHoodPitch,
                // () -> Degrees.of(hoodPos.get()),
                container.getAimSubsystem()::getRobotYawRad,
                container.getAimSubsystem()::getRobotYawRateRadPerSec,
                // () -> RobotState.getInstance().getRobotPose().getRotation(),
                xSupplier,
                ySupplier))
        .withName("Auto Aim At Target");
  }
}
