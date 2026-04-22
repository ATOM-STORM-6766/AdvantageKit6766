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

public final class H1Auto {

  private static final String trajectoryName = "h1";

  private H1Auto() {}

  public static Command create(
      AutoFactory autoFactory, RobotContainer container, boolean shouldMirror) {
    var trj = ChoreoTraj.ALL_TRAJECTORIES.get(trajectoryName);
    var drive = container.getDrive();
    var flywheel = container.getFlywheel();
    var feeder = container.getFeeder();
    var hood = container.getHood();
    var intake = container.getIntake();

    return Commands.sequence(
        GamePieceCommands.resetMechanisms(intake, hood),
        GamePieceCommands.runIntake(intake),
        AutoDriveCommands.resetOdometry(drive, trj.initialPoseBlue(), shouldMirror),
        FollowTrajectory.create(autoFactory, drive, trj, 0, shouldMirror),
        AutoDriveCommands.zeroChassisSpeeds(drive),
        Commands.race(
            GamePieceCommands.feedAndStow(intake, feeder, 2),
            ShootingCommands.autoAimShot(
                container, () -> AllianceFlipUtil.apply(FieldConstants.Hub.topCenterPoint), 6)),
        ShootingCommands.stopShooting(flywheel, feeder, intake));
  }
}
