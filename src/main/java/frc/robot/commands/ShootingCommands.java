package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.flywheel.Flywheel;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.intake.Intake;
import java.util.function.Supplier;

public final class ShootingCommands {
  private ShootingCommands() {}

  public static Command autoAimShot(
      RobotContainer container, Supplier<Translation3d> targetSupplier, double timeoutSeconds) {
    return AimCommand.autoAimAtTarget(container, targetSupplier, () -> 0.0, () -> 0.0)
        .withTimeout(timeoutSeconds)
        .withName("Auto Aim Shot");
  }

  public static Command shootAtFixedPosition(
      Flywheel flywheel, Hood hood, Command feedCommand, double timeoutSeconds) {
    return Commands.parallel(
            flywheel.setVelocity(() -> RotationsPerSecond.of(49)),
            hood.positionSetpointCommand(() -> Degrees.of(30)),
            feedCommand)
        .withTimeout(timeoutSeconds)
        .withName("Shoot At Fixed Position");
  }

  public static Command stopShooting(Flywheel flywheel, Feeder feeder, Intake intake) {
    return Commands.parallel(
            flywheel.stopCommand(),
            feeder.stopCommand(),
            intake.setPosCommand(() -> Degrees.of(0)).andThen(intake.stopCommand()))
        .withName("Stop Shooting");
  }
}
