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

public final class BLV2Auto {

  private BLV2Auto() {}

  public static Command create(
      AutoFactory autoFactory, RobotContainer container, boolean shouldMirror) {
    var trj = ChoreoTraj.BL_V2;
    var drive = container.getDrive();
    var flywheel = container.getFlywheel();
    var feeder = container.getFeeder();
    var hood = container.getHood();
    var intake = container.getIntake();

    return Commands.sequence(
        GamePieceCommands.resetMechanisms(intake, hood),
        GamePieceCommands.runIntake(intake, true),
        AutoDriveCommands.resetOdometry(drive, trj.initialPoseBlue(), shouldMirror),

        // Seg 0 (~3.4s): sweep right side to collect notes
        FollowTrajectory.create(autoFactory, drive, trj, 0, shouldMirror),

        // Seg 1 (~1.6s): return to left-side shooting position (heading ~130°)
        FollowTrajectory.create(autoFactory, drive, trj, 1, shouldMirror),

        // First shot
        Commands.race(
            GamePieceCommands.feedAndStow(intake, feeder, 0.5),
            ShootingCommands.autoAimShot(
                container,
                () -> AllianceFlipUtil.apply(FieldConstants.Position.hubCenterPoint),
                3)),
        ShootingCommands.stopShooting(flywheel, feeder, intake),

        // Seg 2 (~1.1s): short transit to upper-left; redeploy intake on the way
        GamePieceCommands.runIntake(intake, true),
        FollowTrajectory.create(autoFactory, drive, trj, 2, shouldMirror),

        // // Seg 3 (~4.5s): long sweep to collect notes from lower-right area
        // FollowTrajectory.create(autoFactory, drive, trj, 3, shouldMirror),

        // Seg 4 (~1.6s): return to shooting position (heading ~130°)
        FollowTrajectory.create(autoFactory, drive, trj, 3, shouldMirror),
        AutoDriveCommands.zeroChassisSpeeds(drive),

        // Second shot
        Commands.race(
            GamePieceCommands.feedAndStow(intake, feeder, 0.5),
            ShootingCommands.autoAimShot(
                container,
                () -> AllianceFlipUtil.apply(FieldConstants.Position.hubCenterPoint),
                3)),
        ShootingCommands.stopShooting(flywheel, feeder, intake),

        // Seg 5 (~2.8s): drive to final parking position
        FollowTrajectory.create(autoFactory, drive, trj, 4, shouldMirror));
  }
}
