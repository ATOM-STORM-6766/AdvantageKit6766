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

public final class P6Auto {
  private P6Auto() {}

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
        autoFactory.trajectoryCmd("p6", 0).beforeStarting(autoFactory.resetOdometry("p6")),
        AutoDriveCommands.followPoint(
            drive, new Pose2d(7.8589396476745605, 7.029058456420898, Rotation2d.fromDegrees(-90))),
        autoFactory.trajectoryCmd("p6", 1),
        AutoDriveCommands.followPoint(
            drive, new Pose2d(7.532529354095459, 5.936294078826904, Rotation2d.fromDegrees(-90))),
        autoFactory.trajectoryCmd("p6", 2),
        AutoDriveCommands.followPoint(
            drive,
            new Pose2d(
                3.5140202045440674,
                5.620737552642822,
                Rotation2d.fromRadians(-0.9667225715055064))),
        AutoDriveCommands.stopDrive(drive),
        Commands.parallel(
                ShootingCommands.autoAimShot(
                    container, () -> AllianceFlipUtil.apply(FieldConstants.Hub.topCenterPoint), 6),
                GamePieceCommands.feedAndStow(intake, feeder, 1))
            .withTimeout(6),
        ShootingCommands.stopShooting(flywheel, feeder, intake)
            .alongWith(
                AutoDriveCommands.followPoint(
                    drive,
                    new Pose2d(
                        5.5501532554626465,
                        5.673675060272217,
                        Rotation2d.fromRadians(-1.2068177253516055)))),
        GamePieceCommands.runIntake(intake).withName("Auto Intake Reload"),
        AutoDriveCommands.followPoint(
            drive, new Pose2d(7.664889812469482, 5.1983232498168945, Rotation2d.fromDegrees(-90))));
  }
}
