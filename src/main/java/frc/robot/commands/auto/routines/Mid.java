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

public final class Mid {

  private Mid() {}

  public static Command create(
      AutoFactory autoFactory, RobotContainer container, boolean shouldMirror) {
    var l1 = ChoreoTraj.MID;

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
            GamePieceCommands.feedAndStow(intake, feeder, flywheel, 0.5),
            ShootingCommands.autoAimShot(
                container,
                () -> AllianceFlipUtil.apply(FieldConstants.Position.hubCenterPoint),
                3)),
        ShootingCommands.stopShooting(flywheel, feeder, intake),
        FollowTrajectory.create(autoFactory, drive, l1, 2, shouldMirror));
  }
}
