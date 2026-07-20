package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.AimCommand;
import frc.robot.commands.BlinkToTrench;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.GamePieceCommands;
import frc.robot.commands.MoveToTrench;
import frc.robot.commands.PassCommand;
import frc.robot.commands.TuningCommand;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.flywheel.Flywheel;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.light.Light;
import frc.robot.util.AllianceFlipUtil;

public class RobotControl {
  private final RobotContainer robotContainer;
  private final Drive drive;
  private final Flywheel flywheel;
  private final Feeder feeder;
  private final Intake intake;
  private final Hood hood;
  private final Light light;
  private final CommandXboxController controller;
  private final CommandPS5Controller operator;

  public RobotControl(RobotContainer robotContainer) {
    this.robotContainer = robotContainer;
    this.drive = robotContainer.getDrive();
    this.flywheel = robotContainer.getFlywheel();
    this.feeder = robotContainer.getFeeder();
    this.intake = robotContainer.getIntake();
    this.hood = robotContainer.getHood();
    this.light = robotContainer.getLight();
    this.controller = robotContainer.getController();
    this.operator = robotContainer.getOperator();
  }

  public void configure() {
    // 配置驾驶手柄相关绑定。
    configureControllerBindings();

    // 配置操作手柄相关绑定。
    configureOperatorBindings();

    // 瞄准中飞轮就绪时自动切换灯光为 READY 状态。
    new Trigger(() -> controller.a().getAsBoolean() && flywheel.isAtTargetVelocity())
        .onTrue(light.setStateCommand(Light.LightState.READY))
        .onFalse(
            Commands.either(
                light.setStateCommand(Light.LightState.AIM),
                light.setStateCommand(Light.LightState.ALLIANCE),
                controller.a()));
  }

  private void configureControllerBindings() {
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> -controller.getRightX()));

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
        .whileTrue(GamePieceCommands.runIntake(intake, false))
        .onFalse(GamePieceCommands.stopIntakeAndFeeder(intake, feeder));

    controller
        .a()
        .whileTrue(
            light
                .setStateCommand(Light.LightState.AIM)
                .andThen(
                    AimCommand.autoAimAtTarget(
                        robotContainer,
                        () -> AllianceFlipUtil.apply(FieldConstants.Position.hubCenterPoint),
                        () -> -controller.getLeftY(),
                        () -> -controller.getLeftX())))
        .onFalse(
            Commands.parallel(
                flywheel.stopCommand(),
                feeder.stopCommand(),
                intake.stopCommand(),
                light.setStateCommand(Light.LightState.ALLIANCE)));

    controller
        .x()
        .or(operator.square())
        .whileTrue(GamePieceCommands.feedAndStow(intake, feeder, flywheel, 0))
        .onFalse(GamePieceCommands.clearAndStopFeed(intake, feeder));

    controller
        .y()
        .whileTrue(GamePieceCommands.reverseFeed(intake, feeder))
        .onFalse(GamePieceCommands.stopIntakeAndFeeder(intake, feeder));

    // 配置需要自动对准或定点移动的驾驶绑定。
    controller
        .pov(90)
        .whileTrue(
            BlinkToTrench.createRight(drive)
                .raceWith(flywheel.setVelocity(() -> RotationsPerSecond.of(31)))
                .andThen(
                    Commands.parallel(
                        Commands.run(drive::stopWithX, drive),
                        AimCommand.noMoveShootAtTarget(
                            robotContainer,
                            () -> AllianceFlipUtil.apply(FieldConstants.Position.hubCenterPoint)))))
        .onFalse(Commands.parallel(flywheel.stopCommand(), feeder.stopCommand()));

    controller
        .pov(270)
        .whileTrue(
            BlinkToTrench.createLeft(drive)
                // .raceWith(flywheel.setVelocity(() -> RotationsPerSecond.of(31)))
                .andThen(
                    Commands.parallel(
                        Commands.run(drive::stopWithX, drive),
                        AimCommand.noMoveShootAtTarget(
                            robotContainer,
                            () -> AllianceFlipUtil.apply(FieldConstants.Position.hubCenterPoint)))))
        .onFalse(Commands.parallel(flywheel.stopCommand(), feeder.stopCommand()));

    controller
        .leftTrigger()
        .whileTrue(
            MoveToTrench.createLeft(drive)
                .andThen(
                    DriveCommands.joystickDrive(
                        drive,
                        () -> -controller.getLeftY(),
                        () -> 0.0,
                        () -> -controller.getRightX())));

    controller
        .rightTrigger()
        .whileTrue(
            MoveToTrench.createRight(drive)
                .andThen(
                    DriveCommands.joystickDrive(
                        drive,
                        () -> -controller.getLeftY(),
                        () -> 0.0,
                        () -> -controller.getRightX())));
  }

  private void configureOperatorBindings() {
    operator
        .triangle()
        .whileTrue(
            Commands.parallel(
                Commands.startEnd(
                    () ->
                        robotContainer
                            .getAimSubsystem()
                            .setTargetSupplier(
                                () ->
                                    AllianceFlipUtil.apply(FieldConstants.Position.hubCenterPoint)),
                    robotContainer.getAimSubsystem()::clearTargetSupplier,
                    robotContainer.getAimSubsystem()),
                TuningCommand.tuningShootAtPosition(
                    flywheel,
                    hood,
                    feeder,
                    drive,
                    robotContainer.getAimSubsystem()::getRobotYawRad,
                    5)))
        .onFalse(
            Commands.parallel(
                flywheel.stopCommand(),
                feeder.stopCommand(),
                hood.positionSetpointCommand(() -> Degrees.of(15))));

    operator.circle().onTrue(TuningCommand.tuningIntakeSlowStow(intake));

    // operator.L1().whileTrue(TuningCommand.tuningIntakeSetPos(intake,
    // Intake.Position.DEPLOYED));

    // operator.R1().whileTrue(TuningCommand.tuningIntakeRoll(intake)).onFalse(intake.stopCommand());

    operator.L2().whileTrue(intake.unProtectedExtend()).onFalse(intake.stopPosition());

    // 配置传球相关操作手绑定。
    operator
        .cross()
        .whileTrue(
            PassCommand.passAtTarget(
                robotContainer,
                () -> {
                  var target = new Translation3d(FieldConstants.Position.passPoint);
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
