package frc.robot.commands.auto.routines;

import choreo.auto.AutoFactory;
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

public final class P4Auto {

  private static final String trajectoryName = "p4";

  private P4Auto() {}

  public static Command create(
      AutoFactory autoFactory, RobotContainer container, boolean shouldMirror) {
    var trj = ChoreoTraj.ALL_TRAJECTORIES.get(trajectoryName);
    var drive = container.getDrive();
    var flywheel = container.getFlywheel();
    var feeder = container.getFeeder();
    var hood = container.getHood();
    var intake = container.getIntake();

    Command feedCommand = GamePieceCommands.feedAndStow(intake, feeder, 1);

    return Commands.sequence(
        GamePieceCommands.resetMechanisms(intake, hood),
        GamePieceCommands.runIntake(intake),
        ShootingCommands.shootAtFixedPosition(flywheel, hood, feedCommand, 3.5),
        Commands.parallel(flywheel.stopCommand(), feeder.stopCommand()),
        intake.setSlowStowCommand(),
        AutoDriveCommands.resetOdometry(drive, trj.initialPoseBlue(), shouldMirror),
        FollowTrajectory.create(autoFactory, drive, trj, 0, shouldMirror),
        FollowTrajectory.create(autoFactory, drive, trj, 1, shouldMirror),
        FollowTrajectory.create(autoFactory, drive, trj, 2, shouldMirror),

        // 自动瞄准并射击
        Commands.parallel(
                GamePieceCommands.feedAndStow(intake, feeder, 0.5),
                ShootingCommands.autoAimShot(
                    container, () -> AllianceFlipUtil.apply(FieldConstants.Hub.topCenterPoint), 6))
            .withTimeout(6),

        // 停止射击
        ShootingCommands.stopShooting(flywheel, feeder, intake));
  }
}
