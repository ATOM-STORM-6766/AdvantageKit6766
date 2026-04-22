package frc.robot;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.auto.routines.H1Auto;
import frc.robot.commands.auto.routines.P2Auto;
import frc.robot.commands.auto.routines.P4Auto;
import frc.robot.commands.auto.routines.P6Auto;
import frc.robot.commands.auto.routines.TestAuto;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.LoggedTunableBoolean;
import org.littletonrobotics.junction.Logger;

public class AutoModeSelector {
  private final RobotContainer robotContainer;
  private final Drive drive;
  private AutoChooser autoChooser = new AutoChooser();
  private static final LoggedTunableBoolean shouldMirror =
      new LoggedTunableBoolean("shouldMirror", false);

  private AutoFactory autoFactory;

  public AutoModeSelector(RobotContainer robotContainer) {
    this.robotContainer = robotContainer;
    this.drive = robotContainer.getDrive();

    SmartDashboard.putData("Auto Choices", autoChooser);
  }

  public void update() {
    boolean mirrorChanged = shouldMirror.hasChanged(shouldMirror.hashCode());
    if (autoFactory != null && !mirrorChanged) {
      Logger.recordOutput("Robot/Auto/ChooserReady", true);
      return;
    }

    if (mirrorChanged) {
      autoChooser = new AutoChooser();
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

    CommandScheduler.getInstance().schedule(autoFactory.warmupCmd());

    autoChooser.addCmd("H1", () -> H1Auto.create(autoFactory, robotContainer, shouldMirror.get()));
    autoChooser.addCmd("P2", () -> P2Auto.create(autoFactory, robotContainer, shouldMirror.get()));
    autoChooser.addCmd("P4", () -> P4Auto.create(autoFactory, robotContainer, shouldMirror.get()));
    autoChooser.addCmd("P6", () -> P6Auto.create(autoFactory, robotContainer, shouldMirror.get()));

    autoChooser.addCmd(
        "test", () -> TestAuto.create(autoFactory, robotContainer, shouldMirror.get()));

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
