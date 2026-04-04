// 版权 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// 本程序是自由软件；您可以根据自由软件基金会发布的 GNU 通用公共许可证
// 第 3 版的条款进行再发布和/或修改，或在本项目根目录中查阅该许可证。
//
// 本程序的发布目的是希望它有用，但不提供任何担保；
// 甚至没有适销性或特定用途适用性的默示担保。
// 详细信息请参阅 GNU 通用公共许可证。

package frc.robot.match;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.FieldConstants;
import frc.robot.RobotContainer;
import frc.robot.commands.AimCommand;
import frc.robot.commands.FollowPoint;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.flywheel.Flywheel;
import frc.robot.subsystems.flywheel.FlywheelIO.FlywheelSetpoint;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.intake.Intake;
import frc.robot.util.AllianceFlipUtil;
import org.littletonrobotics.junction.Logger;

public class AutoConfigurator {
  private final RobotContainer robotContainer;
  private final Drive drive;
  private final Flywheel flywheel;
  private final Feeder feeder;
  private final Hood hood;
  private final Intake intake;

  public AutoConfigurator(
      RobotContainer robotContainer,
      Drive drive,
      Flywheel flywheel,
      Feeder feeder,
      Hood hood,
      Intake intake) {
    this.robotContainer = robotContainer;
    this.drive = drive;
    this.flywheel = flywheel;
    this.feeder = feeder;
    this.hood = hood;
    this.intake = intake;
  }

  public AutoFactory configure(AutoChooser autoChooser) {
    var autoFactory =
        new AutoFactory(
            drive::getPose,
            drive::setPose,
            drive::followTrajectory,
            DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
            drive,
            (trajectory, isFinsh) -> {
              Logger.recordOutput(
                  "Odometry/Trajectory",
                  DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red
                      ? trajectory.flipped().getPoses()
                      : trajectory.getPoses());
              Logger.recordOutput("Odometry/TrajectoryIsFinished", isFinsh);
            });

    var resetCmd =
        Commands.parallel(intake.resetToLimitCommand().alongWith(hood.resetToLimitCommand()));
    var intakeCmd =
        Commands.parallel(
                intake.setIntakeVelocityCommand(Volts.of(10)),
                intake.setPosCommand(() -> Degrees.of(0)))
            .withName("Intake");
    var feedCmd =
        Commands.race(
                feeder.setFeederVelocityCommand(
                    () -> RotationsPerSecond.of(48), () -> RotationsPerSecond.of(90)),
                intake.WaveIntakeCommand())
            .beforeStarting(Commands.waitSeconds(1));
    var shootCmd =
        AimCommand.autoAimAtTarget(
            robotContainer,
            () -> AllianceFlipUtil.apply(FieldConstants.Hub.topCenterPoint),
            () -> 0.0,
            () -> 0.0);
    var stopShootCmd =
        Commands.parallel(
            flywheel.stopCommand(),
            feeder.stopCommand(),
            intake.setPosCommand(() -> Degrees.of(0)).andThen(intake.stopCommand()));
    var shootAtPosition =
        Commands.parallel(
                flywheel.setVelocity(
                    () ->
                        new FlywheelSetpoint(
                            RotationsPerSecond.of(49),
                            RotationsPerSecond.of(49),
                            RotationsPerSecond.of(49))),
                hood.positionSetpointCommand(() -> Degrees.of(30)),
                feedCmd)
            .withTimeout(3.5);
    var pose1Cmd =
        new FollowPoint(
            drive,
            () ->
                AllianceFlipUtil.apply(
                    new Pose2d(
                        7.8589396476745605, 7.029058456420898, Rotation2d.fromDegrees(-90))));
    var pose2Cmd =
        new FollowPoint(
            drive,
            () ->
                AllianceFlipUtil.apply(
                    new Pose2d(7.532529354095459, 5.936294078826904, Rotation2d.fromDegrees(-90))));
    var pose3Cmd =
        new FollowPoint(
            drive,
            () ->
                AllianceFlipUtil.apply(
                    new Pose2d(
                        3.5140202045440674,
                        5.620737552642822,
                        Rotation2d.fromRadians(-0.9667225715055064))));

    var p2Cmd =
        autoFactory
            .trajectoryCmd("p2")
            .beforeStarting(autoFactory.resetOdometry("p2"))
            .withName("P2 Trajectory");
    var p3_0Cmd =
        autoFactory
            .trajectoryCmd("p3", 0)
            .beforeStarting(autoFactory.resetOdometry("p3"))
            .withName("P3 Trajectory");
    var p3_1Cmd = autoFactory.trajectoryCmd("p3", 1).withName("P3 Trajectory Part 2");
    var p3_2Cmd = autoFactory.trajectoryCmd("p3", 2).withName("P3 Trajectory Part 3");
    var p4_0Cmd =
        autoFactory
            .trajectoryCmd("p4", 0)
            .beforeStarting(autoFactory.resetOdometry("p4"))
            .withName("P4 Trajectory Part 1");
    var p4_1Cmd = autoFactory.trajectoryCmd("p4", 1).withName("P4 Trajectory Part 2");
    var p4_2Cmd = autoFactory.trajectoryCmd("p4", 2).withName("P4 Trajectory Part 3");

    var p6_0Cmd =
        autoFactory
            .trajectoryCmd("p6", 0)
            .beforeStarting(autoFactory.resetOdometry("p6"))
            .withName("P6 Trajectory Part 1");
    var p6_1Cmd = autoFactory.trajectoryCmd("p6", 1).withName("P6 Trajectory Part 2");
    var p6_2Cmd = autoFactory.trajectoryCmd("p6", 2).withName("P6 Trajectory Part 3");

    var p6_0_mirrorCmd =
        autoFactory
            .trajectoryCmd("p6_mirror", 0)
            .beforeStarting(autoFactory.resetOdometry("p6_mirror"))
            .withName("P6 Mirror Trajectory Part 1");
    var p6_1_mirrorCmd =
        autoFactory.trajectoryCmd("p6_mirror", 1).withName("P6 Mirror Trajectory Part 2");
    var p6_2_mirrorCmd =
        autoFactory.trajectoryCmd("p6_mirror", 2).withName("P6 Mirror Trajectory Part 3");

    autoChooser.addCmd(
        "P2",
        () ->
            Commands.sequence(
                resetCmd,
                intakeCmd,
                p2Cmd,
                Commands.runOnce(() -> drive.runVelocity(new ChassisSpeeds()), drive),
                Commands.race(
                    Commands.race(
                            feeder.setFeederVelocityCommand(
                                () -> RotationsPerSecond.of(48), () -> RotationsPerSecond.of(90)),
                            intake.WaveIntakeCommand())
                        .beforeStarting(Commands.waitSeconds(2)),
                    shootCmd.withTimeout(6)),
                stopShootCmd));

    autoChooser.addCmd(
        "P3",
        () ->
            Commands.sequence(
                resetCmd,
                intakeCmd,
                p3_0Cmd,
                new FollowPoint(
                    drive,
                    () ->
                        AllianceFlipUtil.apply(
                            new Pose2d(
                                0.8837900161743164,
                                5.983599662780762,
                                Rotation2d.fromDegrees(180)))),
                new FollowPoint(
                    drive,
                    () ->
                        AllianceFlipUtil.apply(
                            new Pose2d(
                                1.3281044721603394,
                                5.964677333831787,
                                Rotation2d.fromDegrees(180)))),
                p3_1Cmd,
                Commands.runOnce(() -> drive.runVelocity(new ChassisSpeeds()), drive),
                Commands.parallel(
                    Commands.race(
                            feeder.setFeederVelocityCommand(
                                () -> RotationsPerSecond.of(48), () -> RotationsPerSecond.of(90)),
                            intake.WaveIntakeCommand())
                        .beforeStarting(Commands.waitSeconds(2)),
                    shootCmd.withTimeout(6)),
                stopShootCmd,
                p3_2Cmd,
                new FollowPoint(
                    drive,
                    () ->
                        AllianceFlipUtil.apply(
                            new Pose2d(
                                1.505340428352356,
                                3.7539849281311035,
                                Rotation2d.fromDegrees(180))))));

    autoChooser.addCmd(
        "P4",
        () ->
            Commands.sequence(
                resetCmd,
                intakeCmd,
                shootAtPosition,
                Commands.parallel(flywheel.stopCommand(), feeder.stopCommand()),
                intake.setPosCommand(() -> Degrees.of(0)),
                p4_0Cmd,
                new FollowPoint(
                    drive,
                    () ->
                        AllianceFlipUtil.apply(
                            new Pose2d(
                                7.664889812469482,
                                5.9223713874816895,
                                Rotation2d.fromDegrees(-90)))),
                p4_1Cmd,
                new FollowPoint(
                    drive,
                    () ->
                        AllianceFlipUtil.apply(
                            new Pose2d(
                                7.531137466430664,
                                2.8975112438201904,
                                Rotation2d.fromDegrees(-90)))),
                p4_2Cmd,
                new FollowPoint(
                    drive,
                    () ->
                        AllianceFlipUtil.apply(
                            new Pose2d(
                                2.695476770401001,
                                2.67116117477417,
                                Rotation2d.fromRadians(0.6565357014663004)))),
                Commands.parallel(
                        AimCommand.autoAimAtTarget(
                            robotContainer,
                            () -> AllianceFlipUtil.apply(FieldConstants.Hub.topCenterPoint),
                            () -> 0.0,
                            () -> 0.0),
                        Commands.parallel(
                                feeder.setFeederVelocityCommand(
                                    () -> RotationsPerSecond.of(48),
                                    () -> RotationsPerSecond.of(90)),
                                intake.WaveIntakeCommand())
                            .beforeStarting(Commands.waitSeconds(1)))
                    .withTimeout(6),
                stopShootCmd));

    autoChooser.addCmd(
        "P6",
        () ->
            Commands.sequence(
                resetCmd,
                intakeCmd,
                p6_0Cmd,
                pose1Cmd,
                p6_1Cmd,
                pose2Cmd,
                p6_2Cmd,
                pose3Cmd,
                Commands.runOnce(drive::stop, drive),
                Commands.parallel(
                        AimCommand.autoAimAtTarget(
                            robotContainer,
                            () -> AllianceFlipUtil.apply(FieldConstants.Hub.topCenterPoint),
                            () -> 0.0,
                            () -> 0.0),
                        Commands.parallel(
                                feeder.setFeederVelocityCommand(
                                    () -> RotationsPerSecond.of(48),
                                    () -> RotationsPerSecond.of(90)),
                                intake.WaveIntakeCommand())
                            .beforeStarting(Commands.waitSeconds(1)))
                    .withTimeout(6),
                stopShootCmd.alongWith(
                    new FollowPoint(
                        drive,
                        () ->
                            AllianceFlipUtil.apply(
                                new Pose2d(
                                    5.5501532554626465,
                                    5.673675060272217,
                                    Rotation2d.fromRadians(-1.2068177253516055))))),
                Commands.parallel(
                        intake.setIntakeVelocityCommand(Volts.of(10)),
                        intake.setPosCommand(() -> Degrees.of(0)))
                    .withName("Intake"),
                new FollowPoint(
                    drive,
                    () ->
                        AllianceFlipUtil.apply(
                            new Pose2d(
                                7.664889812469482,
                                5.1983232498168945,
                                Rotation2d.fromDegrees(-90))))));

    autoChooser.addCmd(
        "P6_mirror",
        () ->
            Commands.sequence(
                resetCmd,
                intakeCmd,
                p6_0_mirrorCmd,
                new FollowPoint(
                    drive,
                    () ->
                        AllianceFlipUtil.apply(
                            new Pose2d(
                                7.664889812469482,
                                2.1466286125183114,
                                Rotation2d.fromDegrees(90)))),
                p6_1_mirrorCmd,
                new FollowPoint(
                    drive,
                    () ->
                        AllianceFlipUtil.apply(
                            new Pose2d(
                                7.664889812469482,
                                2.1466286125183114,
                                Rotation2d.fromDegrees(90)))),
                p6_2_mirrorCmd,
                new FollowPoint(
                    drive,
                    () ->
                        AllianceFlipUtil.apply(
                            new Pose2d(
                                3.5140202045440674,
                                2.4482624473571786,
                                Rotation2d.fromRadians(0.9667225715055064)))),
                Commands.runOnce(drive::stop, drive),
                Commands.parallel(
                        AimCommand.autoAimAtTarget(
                            robotContainer,
                            () -> AllianceFlipUtil.apply(FieldConstants.Hub.topCenterPoint),
                            () -> 0.0,
                            () -> 0.0),
                        Commands.parallel(
                                feeder.setFeederVelocityCommand(
                                    () -> RotationsPerSecond.of(48),
                                    () -> RotationsPerSecond.of(90)),
                                intake.WaveIntakeCommand())
                            .beforeStarting(Commands.waitSeconds(1)))
                    .withTimeout(6),
                stopShootCmd.alongWith(
                    new FollowPoint(
                        drive,
                        () ->
                            AllianceFlipUtil.apply(
                                new Pose2d(
                                    5.5501532554626465,
                                    2.395324939727784,
                                    Rotation2d.fromRadians(1.2068177253516055))))),
                Commands.parallel(
                        intake.setIntakeVelocityCommand(Volts.of(10)),
                        intake.setPosCommand(() -> Degrees.of(0)))
                    .withName("Intake"),
                new FollowPoint(
                    drive,
                    () ->
                        AllianceFlipUtil.apply(
                            new Pose2d(
                                7.664889812469482,
                                2.1466286125183114,
                                Rotation2d.fromDegrees(90))))));

    autoChooser.addCmd(
        "p2_1",
        () ->
            autoFactory
                .trajectoryCmd("t5", 0)
                .beforeStarting(autoFactory.resetOdometry("t5"))
                .andThen(autoFactory.trajectoryCmd("t5", 1)));

    return autoFactory;
  }
}
