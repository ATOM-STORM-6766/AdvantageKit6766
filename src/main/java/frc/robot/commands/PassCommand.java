package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.RobotContainer;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class PassCommand {
  public static Command prepare(
      RobotContainer container,
      Supplier<Angle> hoodPitchSupplier,
      Supplier<Rotation2d> robotYaw,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier) {
    return Commands.parallel(
            // 启动底盘旋转 (同时允许驾驶员控制 X/Y 平移)
            DriveCommands.joystickDriveAtAngle(
                container.getDrive(), xSupplier, ySupplier, robotYaw),

            // 启动 Hood 旋转到指定角度
            container.getHood().positionSetpointCommand(hoodPitchSupplier),

            // 启动飞轮到指定速度
            container.getFlywheel().setVelocity(container.getPassSubsystem()::getFlywheelSetpoint))
        .withName("Prepare Pass");
  }

  public static Command passAtTarget(
      RobotContainer container,
      Supplier<Translation3d> targetSupplier,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier) {

    return new ParallelCommandGroup(
            // 设置自动目标（冲突时会被清除）
            Commands.startEnd(
                () -> container.getPassSubsystem().setTargetSupplier(targetSupplier),
                container.getPassSubsystem()::clearTargetSupplier,
                container.getPassSubsystem()),

            // 准备自动瞄准时的底盘和 Hood 旋转和飞轮速度
            prepare(
                container,
                container.getPassSubsystem()::getHoodPitch,
                container.getPassSubsystem()::getRobotYawRad,
                xSupplier,
                ySupplier))
        .withName("Pass At Target");
  }
}
