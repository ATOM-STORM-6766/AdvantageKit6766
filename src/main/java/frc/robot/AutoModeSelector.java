package frc.robot;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.auto.routines.BLV2Auto;
import frc.robot.commands.auto.routines.BLV3Auto;
import frc.robot.commands.auto.routines.L123Auto;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.LoggedTunableBoolean;
import org.littletonrobotics.junction.Logger;

public class AutoModeSelector {
  private final RobotContainer robotContainer;
  private final Drive drive;
  private AutoChooser autoChooser = new AutoChooser();
  private static final LoggedTunableBoolean shouldMirror =
      new LoggedTunableBoolean("shouldMirror", true);

  private AutoFactory autoFactory;
  private boolean isMirror = false;

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

    if (autoFactory == null) {
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
    }
    isMirror = shouldMirror.get();

    autoChooser.addCmd(
        mirrorName("BL_V2"), () -> BLV2Auto.create(autoFactory, robotContainer, isMirror));
    autoChooser.addCmd(
        mirrorName("BL_V3"), () -> BLV3Auto.create(autoFactory, robotContainer, isMirror));
    autoChooser.addCmd(
        mirrorName("L123"), () -> L123Auto.create(autoFactory, robotContainer, isMirror));

    SmartDashboard.putData("Auto Choices", autoChooser);
    Logger.recordOutput("Robot/Auto/ChooserReady", true);

    autoChooser.select(isMirror ? "L123 (mirrored)" : "L123");
  }

  public Command selectedCommand() {
    return autoChooser.selectedCommand();
  }

  private String mirrorName(String name) {
    return name + (isMirror ? " (mirrored)" : "");
  }
}
