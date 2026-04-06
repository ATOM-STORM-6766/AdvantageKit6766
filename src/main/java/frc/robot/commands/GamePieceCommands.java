package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.intake.Intake;

public final class GamePieceCommands {
  private GamePieceCommands() {}

  public static Command resetMechanisms(Intake intake, Hood hood) {
    return Commands.parallel(intake.resetToLimitCommand(), hood.resetToLimitCommand())
        .withName("Reset Mechanisms");
  }

  public static Command runIntake(Intake intake) {
    return Commands.parallel(
            intake.setIntakeVelocityCommand(Volts.of(10)),
            intake.setPosCommand(() -> Degrees.of(0)))
        .withName("Run Intake");
  }

  public static Command reverseFeed(Intake intake, Feeder feeder) {
    return Commands.parallel(
            intake.setIntakeVelocityCommand(Volts.of(-10)),
            feeder.setFeederVelocityCommand(
                () -> RotationsPerSecond.of(-90), () -> RotationsPerSecond.of(-40)))
        .withName("Reverse Feed");
  }

  public static Command feedWithWave(Intake intake, Feeder feeder, double delaySeconds) {
    return Commands.race(
            feeder.setFeederVelocityCommand(
                () -> RotationsPerSecond.of(48), () -> RotationsPerSecond.of(90)),
            intake.WaveIntakeCommand())
        .beforeStarting(Commands.waitSeconds(delaySeconds))
        .withName("Feed With Wave");
  }

  public static Command clearAndStopFeed(Intake intake, Feeder feeder) {
    return Commands.parallel(
            feeder
                .setFeederVelocityCommand(
                    () -> RotationsPerSecond.of(-20.0), () -> RotationsPerSecond.of(-80.0))
                .withTimeout(1)
                .andThen(feeder.stopCommand()),
            intake.stopCommand())
        .withName("Clear And Stop Feed");
  }

  public static Command stopIntakeAndFeeder(Intake intake, Feeder feeder) {
    return Commands.parallel(intake.stopCommand(), feeder.stopCommand())
        .withName("Stop Intake And Feeder");
  }

  public static Command stowIntakeAndStop(Intake intake, Feeder feeder) {
    return Commands.parallel(
            feeder.stopCommand(),
            intake.setPosCommand(() -> Degrees.of(0)).andThen(intake.stopCommand()))
        .withName("Stow Intake And Stop");
  }
}
