package frc.robot.commands.auto.routines;

import choreo.auto.AutoFactory;
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

public final class P2Auto {
  private P2Auto() {}

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
        autoFactory.trajectoryCmd("p2").beforeStarting(autoFactory.resetOdometry("p2")),
        AutoDriveCommands.zeroChassisSpeeds(drive),
        Commands.race(
            GamePieceCommands.feedAndStow(intake, feeder, 2),
            ShootingCommands.autoAimShot(
                container, () -> AllianceFlipUtil.apply(FieldConstants.Hub.topCenterPoint), 6)),
        ShootingCommands.stopShooting(flywheel, feeder, intake));
  }
}
