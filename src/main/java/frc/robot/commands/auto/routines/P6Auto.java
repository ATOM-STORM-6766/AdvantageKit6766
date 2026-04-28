package frc.robot.commands.auto.routines;

import choreo.auto.AutoFactory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.ChoreoTraj;
import frc.robot.FieldConstants;
import frc.robot.RobotContainer;
import frc.robot.commands.AutoDriveCommands;
import frc.robot.commands.FollowTrajectory;
import frc.robot.commands.GamePieceCommands;
import frc.robot.commands.ShootingCommands;
import frc.robot.util.AllianceFlipUtil;

public final class P6Auto {

  private static final String trajectoryName = "p6";

  private P6Auto() {}

  public static Command create(
      AutoFactory autoFactory, RobotContainer container, boolean shouldMirror) {
    var trj = ChoreoTraj.ALL_TRAJECTORIES.get(trajectoryName);
    var drive = container.getDrive();
    var flywheel = container.getFlywheel();
    var feeder = container.getFeeder();
    var hood = container.getHood();
    var intake = container.getIntake();

    Pose2d waypoint1 =
        new Pose2d(
            5.5501532554626465, 5.620737552642822, Rotation2d.fromRadians(2.174870082084287));
    Pose2d waypoint2 =
        new Pose2d(7.664889812469482, 5.1983232498168945, Rotation2d.fromDegrees(-90));

    return Commands.sequence(
        GamePieceCommands.resetMechanisms(intake, hood),
        GamePieceCommands.runIntake(intake),
        AutoDriveCommands.resetOdometry(drive, trj.initialPoseBlue(), shouldMirror),
        FollowTrajectory.create(autoFactory, drive, trj, 0, shouldMirror),
        FollowTrajectory.create(autoFactory, drive, trj, 1, shouldMirror),
        FollowTrajectory.create(autoFactory, drive, trj, 2, shouldMirror),
        AutoDriveCommands.stopDrive(drive),

        // 自动瞄准并射击
        Commands.parallel(
                GamePieceCommands.feedAndStow(intake, feeder, 0.5),
                ShootingCommands.autoAimShot(
                    container, () -> AllianceFlipUtil.apply(FieldConstants.Hub.topCenterPoint), 6))
            .withTimeout(6),

        // 停止射击并前往第一个点位
        ShootingCommands.stopShooting(flywheel, feeder, intake)
            .alongWith(AutoDriveCommands.followPoint(drive, waypoint1, shouldMirror)),

        // 边前往第二个点位边尝试吸球（使用 deadline 保证到达点位后结束，或根据业务调整为 raceWith）
        Commands.deadline(
            AutoDriveCommands.followPoint(drive, waypoint2, shouldMirror),
            GamePieceCommands.runIntake(intake).withName("Auto Intake Reload")));
  }
}
