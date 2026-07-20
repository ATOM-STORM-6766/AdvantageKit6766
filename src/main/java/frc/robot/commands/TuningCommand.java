package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.flywheel.Flywheel;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.Intake.Position;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.Supplier;

public final class TuningCommand {
  private static final LoggedTunableNumber intakeFeederRPS =
      new LoggedTunableNumber("TuningShoot/IntakeFeederRPS", 95.0);
  private static final LoggedTunableNumber shooterFeederRPS =
      new LoggedTunableNumber("TuningShoot/ShooterFeederRPS", 68.0);
  private static final LoggedTunableNumber flywheelRPS =
      new LoggedTunableNumber("TuningShoot/FlywheelRPS", 49.0);
  private static final LoggedTunableNumber hoodDegrees =
      new LoggedTunableNumber("TuningShoot/HoodDegrees", 30.0);
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
                        () -> RotationsPerSecond.of(shooterFeederRPS.get()),
                        flywheel)),
            intake.setIntakeVelocityCommand(() -> Volts.of(5.0)),
            flywheel.setVelocity(() -> RotationsPerSecond.of(flywheelRPS.get())),
            hood.positionSetpointCommand(() -> Degrees.of(hoodDegrees.get())),
            Commands.waitSeconds(0.6).andThen(TuningCommand.tuningIntakeSlowStow(intake)))
        .withName("Tuning Shoot");
  }

  /**
   * 与 {@link ShootingCommands#shootAtFixedPosition} 调用方式一致的可调射击指令，额外包含底盘 yaw 轴旋转控制。flywheel 转速、hood
   * 角度、feeder 双路转速均通过 NetworkTables 实时可调。
   *
   * @param headingSupplier 底盘目标朝向（通常来自 AimSubsystem 计算的对准目标角度）
   */
  public static Command tuningShootAtPosition(
      Flywheel flywheel,
      Hood hood,
      Feeder feeder,
      Drive drive,
      Supplier<Rotation2d> headingSupplier,
      double timeoutSeconds) {
    return Commands.parallel(
            DriveCommands.joystickDriveAtAngle(drive, () -> 0.0, () -> 0.0, headingSupplier),
            flywheel.setVelocity(() -> RotationsPerSecond.of(flywheelRPS.get())),
            hood.positionSetpointCommand(() -> Degrees.of(hoodDegrees.get())),
            feeder.setFeederVelocityCommand(
                () -> RotationsPerSecond.of(intakeFeederRPS.get()),
                () -> RotationsPerSecond.of(shooterFeederRPS.get()),
                flywheel))
        .withTimeout(timeoutSeconds)
        .withName("Tuning Shoot At Position");
  }

  public static Command tuningIntakeSlowStow(Intake intake) {
    return intake.setSlowStowCommand().withName("Tuning Intake Slow Stow");
  }

  public static Command tuningIntakeSetPos(Intake intake, Position position) {
    return intake.setPosCommand(position, false).withName("Tuning Intake Set Position");
  }

  public static Command tuningIntakeRoll(Intake intake) {
    return intake
        .setIntakeVelocityCommand(() -> Volts.of(intakeRollVoltage.get()))
        .withName("Tuning Intake Roll");
  }
}
