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

public final class P4Auto {
  private P4Auto() {}

  public static Command create(
      AutoFactory autoFactory,
      RobotContainer container,
      Drive drive,
      Flywheel flywheel,
      Feeder feeder,
      Hood hood,
      Intake intake) {
    Command feedCommand = GamePieceCommands.feedAndStow(intake, feeder, 1);

    return Commands.sequence(
        GamePieceCommands.resetMechanisms(intake, hood),
        GamePieceCommands.runIntake(intake),
        ShootingCommands.shootAtFixedPosition(flywheel, hood, feedCommand, 3.5),
        Commands.parallel(flywheel.stopCommand(), feeder.stopCommand()),
        intake.setSlowStowCommand(),
        autoFactory.trajectoryCmd("p4", 0).beforeStarting(autoFactory.resetOdometry("p4")),
        AutoDriveCommands.followPoint(
            drive, new Pose2d(7.664889812469482, 5.9223713874816895, Rotation2d.fromDegrees(-90))),
        autoFactory.trajectoryCmd("p4", 1),
        AutoDriveCommands.followPoint(
            drive, new Pose2d(7.531137466430664, 2.8975112438201904, Rotation2d.fromDegrees(-90))),
        autoFactory.trajectoryCmd("p4", 2),
        AutoDriveCommands.followPoint(
            drive,
            new Pose2d(
                2.695476770401001, 2.67116117477417, Rotation2d.fromRadians(0.6565357014663004))),
        Commands.parallel(
                ShootingCommands.autoAimShot(
                    container, () -> AllianceFlipUtil.apply(FieldConstants.Hub.topCenterPoint), 6),
                GamePieceCommands.feedAndStow(intake, feeder, 1))
            .withTimeout(6),
        ShootingCommands.stopShooting(flywheel, feeder, intake));
  }
}
