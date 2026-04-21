package frc.robot.commands.auto.routines;

import choreo.auto.AutoFactory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.FieldConstants;
import frc.robot.RobotContainer;
import frc.robot.commands.AutoDriveCommands;
import frc.robot.commands.GamePieceCommands;
import frc.robot.commands.ShootingCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.flywheel.Flywheel;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.intake.Intake;
import frc.robot.util.AllianceFlipUtil;

public final class P3Auto {
  private P3Auto() {}

  public static Command create(
      AutoFactory autoFactory,
      RobotContainer container,
      Drive drive,
      Flywheel flywheel,
      Feeder feeder,
      Hood hood,
      Intake intake) {
    return Commands.sequence(
        GamePieceCommands.resetMechanisms(intake, hood),
        GamePieceCommands.runIntake(intake),
        autoFactory.trajectoryCmd("p3", 0).beforeStarting(autoFactory.resetOdometry("p3")),
        AutoDriveCommands.followPoint(
            drive, new Pose2d(0.8837900161743164, 5.983599662780762, Rotation2d.fromDegrees(180))),
        AutoDriveCommands.followPoint(
            drive, new Pose2d(1.3281044721603394, 5.964677333831787, Rotation2d.fromDegrees(180))),
        autoFactory.trajectoryCmd("p3", 1),
        AutoDriveCommands.zeroChassisSpeeds(drive),
        Commands.parallel(
            GamePieceCommands.feedAndStow(intake, feeder, 2),
            ShootingCommands.autoAimShot(
                container, () -> AllianceFlipUtil.apply(FieldConstants.Hub.topCenterPoint), 6)),
        ShootingCommands.stopShooting(flywheel, feeder, intake),
        autoFactory.trajectoryCmd("p3", 2),
        AutoDriveCommands.followPoint(
            drive, new Pose2d(1.505340428352356, 3.7539849281311035, Rotation2d.fromDegrees(180))));
  }
}
