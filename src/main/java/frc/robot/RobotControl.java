package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.AimCommand;
import frc.robot.commands.AutoDriveCommands;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.FollowPoint;
import frc.robot.commands.GamePieceCommands;
import frc.robot.commands.PassCommand;
import frc.robot.commands.TuningShootCommand;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.flywheel.Flywheel;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.intake.Intake;
import frc.robot.util.AllianceFlipUtil;

public class RobotControl {
  private final RobotContainer robotContainer;
  private final Drive drive;
  private final Flywheel flywheel;
  private final Feeder feeder;
  private final Intake intake;
  private final Hood hood;
  private final CommandXboxController controller;
  private final CommandPS5Controller operator;

  public RobotControl(RobotContainer robotContainer) {
    this.robotContainer = robotContainer;
    this.drive = robotContainer.getDrive();
    this.flywheel = robotContainer.getFlywheel();
    this.feeder = robotContainer.getFeeder();
    this.intake = robotContainer.getIntake();
    this.hood = robotContainer.getHood();
    this.controller = robotContainer.getController();
    this.operator = robotContainer.getOperator();
  }

  public void configure() {
    // 配置驾驶手柄相关绑定。
    configureControllerBindings();

    // 配置操作手柄相关绑定。
    configureOperatorBindings();
  }

  private void configureControllerBindings() {
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

    // 配置 intake、喂料与射球相关驾驶绑定。
    controller
        .rightBumper()
        .whileTrue(GamePieceCommands.runIntake(intake))
        .onFalse(GamePieceCommands.stopIntakeAndFeeder(intake, feeder));

    controller
        .a()
        .whileTrue(
            AimCommand.autoAimAtTarget(
                robotContainer,
                () -> AllianceFlipUtil.apply(FieldConstants.Hub.topCenterPoint),
                () -> -controller.getLeftY(),
                () -> -controller.getLeftX()))
        .onFalse(Commands.parallel(flywheel.stopCommand(), feeder.stopCommand()));

    controller
        .x()
        .or(operator.square())
        .whileTrue(GamePieceCommands.feedAndStow(intake, feeder, 0))
        .onFalse(GamePieceCommands.clearAndStopFeed(intake, feeder));

    controller
        .y()
        .whileTrue(GamePieceCommands.reverseFeed(intake, feeder))
        .onFalse(GamePieceCommands.stopIntakeAndFeeder(intake, feeder));

    // 配置需要自动对准或定点移动的驾驶绑定。
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
            AutoDriveCommands.followPoint(drive, FieldConstants.Hub.blink)
                .raceWith(flywheel.setVelocity(() -> RotationsPerSecond.of(20)))
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
            AutoDriveCommands.followPoint(drive, AllianceFlipUtil.mirror(FieldConstants.Hub.blink))
                .raceWith(flywheel.setVelocity(() -> RotationsPerSecond.of(20)))
                .andThen(
                    Commands.parallel(
                        Commands.run(drive::stopWithX, drive),
                        AimCommand.noMoveShootAtTarget(
                            robotContainer,
                            () -> AllianceFlipUtil.apply(FieldConstants.Hub.topCenterPoint)))))
        .onFalse(Commands.parallel(flywheel.stopCommand(), feeder.stopCommand()));
  }

  private void configureOperatorBindings() {
    operator
        .triangle()
        .whileTrue(TuningShootCommand.tuningShoot(feeder, flywheel, hood))
        .onFalse(
            Commands.parallel(
                flywheel.stopCommand(),
                feeder.stopCommand(),
                hood.positionSetpointCommand(() -> Degrees.of(17))));

    // 配置传球相关操作手绑定。
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
  }
}
