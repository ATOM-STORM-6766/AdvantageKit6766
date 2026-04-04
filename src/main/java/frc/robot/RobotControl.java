package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.AimCommand;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.FollowPoint;
import frc.robot.commands.PassCommand;
import frc.robot.subsystems.clamber.Clamber;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.flywheel.Flywheel;
import frc.robot.subsystems.flywheel.FlywheelIO.FlywheelSetpoint;
import frc.robot.subsystems.intake.Intake;
import frc.robot.util.AllianceFlipUtil;

public class RobotControl {
  private final RobotContainer robotContainer;
  private final Drive drive;
  private final Clamber clamber;
  private final Flywheel flywheel;
  private final Feeder feeder;
  private final Intake intake;
  private final CommandXboxController controller;
  private final CommandPS5Controller operator;

  public RobotControl(
      RobotContainer robotContainer,
      Drive drive,
      Clamber clamber,
      Flywheel flywheel,
      Feeder feeder,
      Intake intake,
      CommandXboxController controller,
      CommandPS5Controller operator) {
    this.robotContainer = robotContainer;
    this.drive = drive;
    this.clamber = clamber;
    this.flywheel = flywheel;
    this.feeder = feeder;
    this.intake = intake;
    this.controller = controller;
    this.operator = operator;
  }

  public void configure() {
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> -controller.getRightX()));
    controller
        .leftBumper()
        .whileTrue(
            DriveCommands.joystickRobotDrive(
                drive,
                () -> -controller.getLeftY(),
                () -> -controller.getLeftX(),
                () -> -controller.getRightX() * 0.9));

    controller
        .rightTrigger()
        .whileTrue(
            DriveCommands.snapToNearest30Degrees(
                drive, () -> -controller.getLeftY(), () -> -controller.getLeftX()));

    controller.back().onTrue(Commands.runOnce(drive::stopWithX, drive));

    controller
        .start()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
                    drive)
                .ignoringDisable(true));

    controller
        .rightBumper()
        .whileTrue(
            Commands.parallel(
                intake.setIntakeVelocityCommand(Volts.of(10)),
                intake.setPosCommand(() -> Degrees.of(0))))
        .onFalse(Commands.parallel(intake.stopCommand(), feeder.stopCommand()));

    controller
        .a()
        .whileTrue(
            Commands.parallel(
                AimCommand.autoAimAtTarget(
                    robotContainer,
                    () -> AllianceFlipUtil.apply(FieldConstants.Hub.topCenterPoint),
                    () -> -controller.getLeftY(),
                    () -> -controller.getLeftX())))
        .onFalse(Commands.parallel(flywheel.stopCommand(), feeder.stopCommand()));

    controller
        .x()
        .or(operator.square())
        .whileTrue(
            Commands.parallel(
                feeder.setFeederVelocityCommand(
                    () -> RotationsPerSecond.of(48), () -> RotationsPerSecond.of(90)),
                intake.WaveIntakeCommand()))
        .onFalse(
            Commands.parallel(
                feeder
                    .setFeederVelocityCommand(
                        () -> RotationsPerSecond.of(-20.0), () -> RotationsPerSecond.of(-80.0))
                    .withTimeout(1)
                    .andThen(feeder.stopCommand()),
                intake.stopCommand()));

    operator.L1().whileTrue(clamber.runPercentCommand(0.6));

    operator.R1().whileTrue(clamber.runPercentCommand(-0.6));

    operator
        .cross()
        .whileTrue(
            PassCommand.passAtTarget(
                robotContainer,
                () -> {
                  var target = new Translation3d(FieldConstants.Hub.passPoint);
                  target = AllianceFlipUtil.apply(target);

                  target =
                      AllianceFlipUtil.apply(drive.getPose()).getTranslation().getY()
                              > FieldConstants.fieldWidth / 2
                          ? target
                          : AllianceFlipUtil.mirror(target);

                  return target;
                },
                () -> -controller.getLeftY(),
                () -> -controller.getLeftX()))
        .onFalse(Commands.parallel(flywheel.stopCommand(), feeder.stopCommand()));

    controller
        .y()
        .whileTrue(
            Commands.parallel(
                intake.setIntakeVelocityCommand(Volts.of(-10)),
                feeder.setFeederVelocityCommand(
                    () -> RotationsPerSecond.of(-90), () -> RotationsPerSecond.of(-40))))
        .onFalse(intake.stopCommand().alongWith(feeder.stopCommand()));

    controller
        .b()
        .whileTrue(
            new FollowPoint(drive, () -> AllianceFlipUtil.apply(FieldConstants.Hub.tower))
                .onlyIf(
                    () ->
                        drive
                                .getPose()
                                .getTranslation()
                                .getDistance(
                                    AllianceFlipUtil.apply(
                                        FieldConstants.Hub.tower.getTranslation()))
                            < 1.6));

    controller
        .pov(90)
        .whileTrue(
            new FollowPoint(drive, () -> AllianceFlipUtil.apply(FieldConstants.Hub.blink))
                .raceWith(
                    flywheel.setVelocity(
                        () ->
                            new FlywheelSetpoint(
                                RotationsPerSecond.of(20),
                                RotationsPerSecond.of(20),
                                RotationsPerSecond.of(20))))
                .andThen(
                    Commands.parallel(
                        Commands.run(drive::stopWithX, drive),
                        AimCommand.noMoveShootAtTarget(
                            robotContainer,
                            () -> AllianceFlipUtil.apply(FieldConstants.Hub.topCenterPoint)))))
        .onFalse(Commands.parallel(flywheel.stopCommand(), feeder.stopCommand()));
    controller
        .pov(270)
        .whileTrue(
            new FollowPoint(
                    drive,
                    () -> AllianceFlipUtil.apply(AllianceFlipUtil.mirror(FieldConstants.Hub.blink)))
                .raceWith(
                    flywheel.setVelocity(
                        () ->
                            new FlywheelSetpoint(
                                RotationsPerSecond.of(20),
                                RotationsPerSecond.of(20),
                                RotationsPerSecond.of(20))))
                .andThen(
                    Commands.parallel(
                        Commands.run(drive::stopWithX, drive),
                        AimCommand.noMoveShootAtTarget(
                            robotContainer,
                            () -> AllianceFlipUtil.apply(FieldConstants.Hub.topCenterPoint)))))
        .onFalse(Commands.parallel(flywheel.stopCommand(), feeder.stopCommand()));
  }
}
