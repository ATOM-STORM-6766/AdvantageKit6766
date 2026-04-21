package frc.robot;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.auto.routines.H1Auto;
import frc.robot.commands.auto.routines.P21Auto;
import frc.robot.commands.auto.routines.P3Auto;
import frc.robot.commands.auto.routines.P4Auto;
import frc.robot.commands.auto.routines.P6Auto;
import frc.robot.commands.auto.routines.P6MirrorAuto;
import frc.robot.commands.auto.routines.TestAuto;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.flywheel.Flywheel;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.intake.Intake;
import frc.robot.util.AllianceFlipUtil;
import org.littletonrobotics.junction.Logger;

public class AutoModeSelector {
  private final RobotContainer robotContainer;
  private final Drive drive;
  private final Flywheel flywheel;
  private final Feeder feeder;
  private final Hood hood;
  private final Intake intake;
  private final AutoChooser autoChooser = new AutoChooser();

  private AutoFactory autoFactory;

  public AutoModeSelector(RobotContainer robotContainer) {
    this.robotContainer = robotContainer;
    this.drive = robotContainer.getDrive();
    this.flywheel = robotContainer.getFlywheel();
    this.feeder = robotContainer.getFeeder();
    this.hood = robotContainer.getHood();
    this.intake = robotContainer.getIntake();

    SmartDashboard.putData("Auto Choices", autoChooser);
  }

  public void update() {
    if (autoFactory != null) {
      Logger.recordOutput("Robot/Auto/ChooserReady", true);
      return;
    }

    if (!AllianceFlipUtil.isAllianceKnown()) {
      Logger.recordOutput("Robot/Auto/ChooserReady", false);
      return;
    }

    autoFactory =
        new AutoFactory(
            drive::getPose,
            drive::setPose,
            drive::followTrajectory,
            AllianceFlipUtil.shouldFlip(),
            drive,
            (trajectory, isFinished) -> {
              Logger.recordOutput(
                  "Odometry/Trajectory",
                  AllianceFlipUtil.shouldFlip()
                      ? trajectory.flipped().getPoses()
                      : trajectory.getPoses());
              Logger.recordOutput("Odometry/TrajectoryIsFinished", isFinished);
            });

    autoChooser.addCmd(
        "H1",
        () -> H1Auto.create(autoFactory, robotContainer, drive, flywheel, feeder, hood, intake));
    autoChooser.addCmd(
        "P3",
        () -> P3Auto.create(autoFactory, robotContainer, drive, flywheel, feeder, hood, intake));
    autoChooser.addCmd(
        "P4",
        () -> P4Auto.create(autoFactory, robotContainer, drive, flywheel, feeder, hood, intake));
    autoChooser.addCmd(
        "P6",
        () -> P6Auto.create(autoFactory, robotContainer, drive, flywheel, feeder, hood, intake));
    autoChooser.addCmd(
        "P6_mirror",
        () ->
            P6MirrorAuto.create(
                autoFactory, robotContainer, drive, flywheel, feeder, hood, intake));
    autoChooser.addCmd("p2_1", () -> P21Auto.create(autoFactory));

    autoChooser.addCmd(
        "test",
        () -> TestAuto.create(autoFactory, robotContainer, drive, flywheel, feeder, hood, intake));

    autoChooser.addCmd("sysidDf", () -> drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addCmd("sysidDb", () -> drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    autoChooser.addCmd("sysidQf", () -> drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addCmd("sysidQb", () -> drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));

    SmartDashboard.putData("Auto Choices", autoChooser);
    Logger.recordOutput("Robot/Auto/ChooserReady", true);
  }

  public Command selectedCommand() {
    return autoChooser.selectedCommand();
  }
}
