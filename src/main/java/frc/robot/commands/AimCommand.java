package frc.robot.commands;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.RobotContainer;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class AimCommand {
  public static Command prepare(
      RobotContainer container,
      DoubleSupplier hoodPitchRadSupplier,
      Supplier<Rotation2d> robotYawRadSupplier,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier) {
    return new ParallelCommandGroup(
            // 启动底盘旋转
            DriveCommands.joystickDriveAtAngle(
                container.getDrive(), xSupplier, ySupplier, robotYawRadSupplier),

            // 启动 Hood 旋转
            container.getHood().positionSetpointCommand(hoodPitchRadSupplier),

            // 启动 Flywheel 旋转
            Commands.sequence(
                // 先反转喂料 0.3s，清理卡球
                container
                    .getFeeder()
                    .setFeederVelocityCommand(
                        () -> RotationsPerSecond.of(-5), () -> RotationsPerSecond.of(0.0))
                    .withTimeout(0.3)
                    .andThen(container.getFeeder().stopCommand()),

                // 设置飞轮速度
                container
                    .getFlywheel()
                    .setVelocity(container.getAimSubsystem()::getFlywheelSetpoint)))
        .withName("Prepare Aim");
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
            Commands.sequence(
                // 等待自瞄解算完成
                Commands.waitUntil(container.getAimSubsystem()::isSetpointValid),

                // 准备自动瞄准时的底盘和 Hood 旋转和飞轮速度
                prepare(
                    container,
                    container.getAimSubsystem()::getHoodPitchRad,
                    container.getAimSubsystem()::getRobotYawRad,
                    xSupplier,
                    ySupplier)))
        .withName("Auto Aim At Target");
  }
}
