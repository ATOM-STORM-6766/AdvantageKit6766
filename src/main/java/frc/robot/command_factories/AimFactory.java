package frc.robot.command_factories;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.hood.HoodConstants;
import frc.robot.subsystems.turret.TurretConstants;
import frc.robot.util.GenericShooterResolverV2;
import frc.robot.util.GenericShooterResolverV2.ShooterSetpointV2;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class AimFactory {
  public static Command sweepTurret(RobotContainer container) {
    double turretMin = TurretConstants.kTurretMinPositionRadians;
    double turretMax = TurretConstants.kTurretMaxPositionRadians;
    double turretMid = (turretMin + turretMax) * 0.5;
    double turretAmp = (turretMax - turretMin) * 0.45;

    double periodSeconds = 6.0;
    double omega = (2.0 * Math.PI) / periodSeconds;

    Timer timer = new Timer();

    return container
        .getTurret()
        .positionSetpointCommand(
            () -> {
              double t = timer.get();
              double target = turretMid + turretAmp * Math.sin(omega * t);
              return MathUtil.clamp(target, turretMin, turretMax);
            },
            () -> 0.0)
        .beforeStarting(timer::start)
        .finallyDo(() -> timer.stop())
        .withName("Sweep Turret");
  }

  public static Command sweepTurretAndHood(RobotContainer container) {
    double turretMin = TurretConstants.kTurretMinPositionRadians;
    double turretMax = TurretConstants.kTurretMaxPositionRadians;
    double turretMid = (turretMin + turretMax) * 0.5;
    double turretAmp = (turretMax - turretMin) * 0.45;

    double hoodMin = HoodConstants.kHoodMinPositionRadians;
    double hoodMax = HoodConstants.kHoodMaxPositionRadians;
    double hoodMid = (hoodMin + hoodMax) * 0.5;
    double hoodAmp = (hoodMax - hoodMin) * 0.45;

    double periodSeconds = 6.0;
    double omega = (2.0 * Math.PI) / periodSeconds;

    Timer timer = new Timer();

    return new ParallelCommandGroup(
            container
                .getTurret()
                .positionSetpointCommand(
                    () -> {
                      double t = timer.get();
                      double target = turretMid + turretAmp * Math.sin(omega * t);
                      return MathUtil.clamp(target, turretMin, turretMax);
                    },
                    () -> 0.0),
            container
                .getHood()
                .positionSetpointCommand(
                    () -> {
                      double t = timer.get();
                      double target = hoodMid + hoodAmp * Math.sin(omega * t);
                      return MathUtil.clamp(target, hoodMin, hoodMax);
                    },
                    () -> 0.0))
        .beforeStarting(timer::start)
        .finallyDo(() -> timer.stop())
        .withName("Sweep Turret And Hood (teleop)");
  }

  public static Command resetAllToLimit(RobotContainer container) {
    return new ParallelCommandGroup(
            container.getHood().resetToLimitCommand(), container.getTurret().resetToLimitCommand())
        .withName("Reset To Limit");
  }

  /**
   * Creates a command that continuously aims at a target using the GenericShooterResolver. The
   * command will continuously update the turret and hood positions based on the current robot pose
   * and speeds, with motion compensation.
   *
   * @param container The robot container
   * @param targetSupplier Supplier for the target position in field coordinates (Translation3d)
   * @return A command that aims the turret and hood at the target
   */
  public static Command aimAtTarget(
      RobotContainer container, Supplier<Translation3d> targetSupplier) {

    // Cache for last valid setpoint
    final ShooterSetpointV2[] lastValidSetpoint = {createDefaultSetpointV2()};

    return new ParallelCommandGroup(
            container
                .getTurret()
                .positionSetpointCommand(
                    () -> {
                      updateSetpoint(container, targetSupplier, lastValidSetpoint);
                      return lastValidSetpoint[0].turretYaw;
                    },
                    () -> lastValidSetpoint[0].turretFeedforward),
            container
                .getHood()
                .positionSetpointCommand(
                    () -> lastValidSetpoint[0].hoodPitch,
                    () -> lastValidSetpoint[0].hoodFeedforward))
        .withName("Aim At Target Direct");
  }

  private static ShooterSetpointV2 createDefaultSetpointV2() {
    ShooterSetpointV2 s = new ShooterSetpointV2();
    s.turretYaw = Math.toRadians(180.0); // Default to facing backward
    s.hoodPitch = Math.toRadians(30.0); // Default to 60 degree pitch (30 degree hood)
    s.turretFeedforward = 0.0;
    s.hoodFeedforward = 0.0;
    s.isValid = false;
    return s;
  }

  private static void updateSetpoint(
      RobotContainer container,
      Supplier<Translation3d> targetSupplier,
      ShooterSetpointV2[] lastValidSetpoint) {

    var robotPose = container.getDrive().getPose();
    var robotSpeeds = container.getDrive().getFieldRelativeChassisSpeeds();
    var target = targetSupplier.get();

    ShooterSetpointV2 setpoint =
        GenericShooterResolverV2.resolve(
            robotPose, robotSpeeds, target, Constants.SHOOTER_CONFIG_V2);

    // Log resolver outputs
    Logger.recordOutput("Aiming/SetpointValid", setpoint.isValid);
    Logger.recordOutput("Aiming/ActualTarget", new Pose3d(target, new Rotation3d()));

    if (setpoint.isValid) {
      lastValidSetpoint[0] = setpoint;

      // Log debug info
      Logger.recordOutput("Aiming/TurretYawDeg", Math.toDegrees(setpoint.turretYaw));
      Logger.recordOutput("Aiming/HoodPitchDeg", Math.toDegrees(setpoint.hoodPitch));
      Logger.recordOutput("Aiming/TimeOfFlight", setpoint.timeOfFlightSeconds);
      Logger.recordOutput("Aiming/FlywheelRps", setpoint.flywheelRps);
      if (setpoint.virtualTarget != null) {
        Logger.recordOutput(
            "Aiming/VirtualTarget", new Pose3d(setpoint.virtualTarget, new Rotation3d()));
      }
    }
  }
}
