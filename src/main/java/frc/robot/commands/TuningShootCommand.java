package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.flywheel.Flywheel;
import frc.robot.subsystems.hood.Hood;
import frc.robot.util.LoggedTunableNumber;

public final class TuningShootCommand {
  private static final LoggedTunableNumber intakeFeederRPS =
      new LoggedTunableNumber("TuningShoot/IntakeFeederRPS", 48.0);
  private static final LoggedTunableNumber shooterFeederRPS =
      new LoggedTunableNumber("TuningShoot/ShooterFeederRPS", 90.0);
  private static final LoggedTunableNumber flywheelRPS =
      new LoggedTunableNumber("TuningShoot/FlywheelRPS", 49.0);
  private static final LoggedTunableNumber hoodDegrees =
      new LoggedTunableNumber("TuningShoot/HoodDegrees", 17.0);

  private TuningShootCommand() {}

  public static Command tuningShoot(Feeder feeder, Flywheel flywheel, Hood hood) {
    return Commands.parallel(
            feeder.setFeederVelocityCommand(
                () -> RotationsPerSecond.of(intakeFeederRPS.get()),
                () -> RotationsPerSecond.of(shooterFeederRPS.get())),
            flywheel.setVelocity(() -> RotationsPerSecond.of(flywheelRPS.get())),
            hood.positionSetpointCommand(() -> Degrees.of(hoodDegrees.get())))
        .withName("Tuning Shoot");
  }
}
