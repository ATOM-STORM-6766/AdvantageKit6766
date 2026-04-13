package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.flywheel.Flywheel;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.Intake.Position;
import frc.robot.util.LoggedTunableNumber;

public final class TuningCommand {
  private static final LoggedTunableNumber intakeFeederRPS =
      new LoggedTunableNumber("TuningShoot/IntakeFeederRPS", 48.0);
  private static final LoggedTunableNumber shooterFeederRPS =
      new LoggedTunableNumber("TuningShoot/ShooterFeederRPS", 90.0);
  private static final LoggedTunableNumber flywheelRPS =
      new LoggedTunableNumber("TuningShoot/FlywheelRPS", 49.0);
  private static final LoggedTunableNumber hoodDegrees =
      new LoggedTunableNumber("TuningShoot/HoodDegrees", 17.0);

  private TuningCommand() {}

  public static Command tuningShoot(Feeder feeder, Flywheel flywheel, Hood hood, Intake intake) {
    return Commands.parallel(
            flywheel
                .waitForVelocity(() -> RotationsPerSecond.of(flywheelRPS.get()), 5)
                .andThen(
                    feeder.setFeederVelocityCommand(
                        () -> RotationsPerSecond.of(intakeFeederRPS.get()),
                        () -> RotationsPerSecond.of(shooterFeederRPS.get()))),
            flywheel.setVelocity(() -> RotationsPerSecond.of(flywheelRPS.get())),
            hood.positionSetpointCommand(() -> Degrees.of(hoodDegrees.get())))
        .withName("Tuning Shoot");
  }

  public static Command tuningIntakeSlowStow(Intake intake) {
    return intake.setSlowStowCommand().withName("Tuning Intake Slow Stow");
  }

  public static Command tuningIntakeSetPos(Intake intake, Position position) {
    return intake.setPosCommand(position).withName("Tuning Intake Set Position");
  }
}
