package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

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
      new LoggedTunableNumber("TuningShoot/IntakeFeederRPS", 30.0);
  private static final LoggedTunableNumber shooterFeederRPS =
      new LoggedTunableNumber("TuningShoot/ShooterFeederRPS", 30.0);
  private static final LoggedTunableNumber flywheelRPS =
      new LoggedTunableNumber("TuningShoot/FlywheelRPS", 30.0);
  private static final LoggedTunableNumber hoodDegrees =
      new LoggedTunableNumber("TuningShoot/HoodDegrees", 17.0);
  private static final LoggedTunableNumber intakeRollVoltage =
      new LoggedTunableNumber("TuningIntakeRoll/IntakeRollVoltage", 9.0);

  private TuningCommand() {}

  public static Command tuningShoot(Feeder feeder, Flywheel flywheel, Hood hood, Intake intake) {
    return Commands.parallel(
            flywheel
                .waitForVelocity(() -> RotationsPerSecond.of(flywheelRPS.get()), 5)
                .andThen(
                    feeder.setFeederVelocityCommand(
                        () -> RotationsPerSecond.of(intakeFeederRPS.get()),
                        () -> RotationsPerSecond.of(shooterFeederRPS.get()))),
            intake.setIntakeVelocityCommand(() -> Volts.of(5.0)),
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

  public static Command tuningIntakeRoll(Intake intake) {
    return intake
        .setIntakeVelocityCommand(() -> Volts.of(intakeRollVoltage.get()))
        .withName("Tuning Intake Roll");
  }
}
