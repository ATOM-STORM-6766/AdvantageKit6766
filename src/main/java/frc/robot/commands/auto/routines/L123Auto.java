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

public final class L123Auto {

  private L123Auto() {}

  public static Command create(
      AutoFactory autoFactory, RobotContainer container, boolean shouldMirror) {
    var l1 = ChoreoTraj.L1;
    var l2 = ChoreoTraj.L2;
    var l3 = ChoreoTraj.L3;

    var drive = container.getDrive();
    var flywheel = container.getFlywheel();
    var feeder = container.getFeeder();
    var hood = container.getHood();
    var intake = container.getIntake();

    return Commands.sequence(
        GamePieceCommands.resetMechanisms(intake, hood),
        GamePieceCommands.runIntake(intake, true),
        AutoDriveCommands.resetOdometry(drive, l1.initialPoseBlue(), shouldMirror),

        // L1
        FollowTrajectory.create(autoFactory, drive, l1, 0, shouldMirror),
        FollowTrajectory.create(autoFactory, drive, l1, 1, shouldMirror),

        // Shot 1
        AutoDriveCommands.zeroChassisSpeeds(drive),
        Commands.race(
            GamePieceCommands.feedAndStow(intake, feeder, 0.5),
            ShootingCommands.autoAimShot(
                container,
                () -> AllianceFlipUtil.apply(FieldConstants.Position.hubCenterPoint),
                3)),
        ShootingCommands.stopShooting(flywheel, feeder, intake),

        // L2
        GamePieceCommands.runIntake(intake, true),
        FollowTrajectory.create(autoFactory, drive, l2, 0, shouldMirror),
        FollowTrajectory.create(autoFactory, drive, l2, 1, shouldMirror),
        FollowTrajectory.create(autoFactory, drive, l2, 2, shouldMirror),

        // Shot 2
        AutoDriveCommands.zeroChassisSpeeds(drive),
        Commands.race(
            GamePieceCommands.feedAndStow(intake, feeder, 0.5),
            ShootingCommands.autoAimShot(
                container,
                () -> AllianceFlipUtil.apply(FieldConstants.Position.hubCenterPoint),
                3)),
        ShootingCommands.stopShooting(flywheel, feeder, intake),

        // L3
        GamePieceCommands.runIntake(intake, true),
        FollowTrajectory.create(autoFactory, drive, l3, shouldMirror));
  }
}
