package frc.robot.command_factories;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.RobotState;
import frc.robot.subsystems.flywheel.FlywheelIO.FlywheelSetpoint;
import frc.robot.subsystems.hood.HoodConstants;
import frc.robot.util.GenericShooterResolver;
import frc.robot.util.GenericShooterResolver.ShooterSetpoint;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class AimFactory {

  public static Command sweepTurretAndHood(RobotContainer container) {

    double hoodMin = HoodConstants.kHoodMinPositionRadians;
    double hoodMax = HoodConstants.kHoodMaxPositionRadians;
    double hoodMid = (hoodMin + hoodMax) * 0.5;
    double hoodAmp = (hoodMax - hoodMin) * 0.45;

    double periodSeconds = 6.0;
    double omega = (2.0 * Math.PI) / periodSeconds;

    Timer timer = new Timer();

    return new ParallelCommandGroup(
            container
                .getHood()
                .positionSetpointCommand(
                    () -> {
                      double t = timer.get();
                      double target = hoodMid + hoodAmp * Math.sin(omega * t);
                      return MathUtil.clamp(target, hoodMin, hoodMax);
                    }))
        .beforeStarting(timer::start)
        .finallyDo(() -> timer.stop())
        .withName("Sweep Turret And Hood (teleop)");
  }

  public static Command resetAllToLimit(RobotContainer container) {
    return new ParallelCommandGroup(
            container.getHood().resetToLimitCommand(), container.getIntake().resetToLimitCommand())
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
    final ShooterSetpoint[] lastValidSetpoint = {createDefaultSetpoint()};

    return new ParallelCommandGroup(
            container.getHood().positionSetpointCommand(() -> lastValidSetpoint[0].hoodPitch),
            container
                .getFlywheel()
                .flywheelSetpointCommand(
                    () ->
                        new FlywheelSetpoint(
                            RotationsPerSecond.of(53.489),
                            RotationsPerSecond.of(53.489),
                            RotationsPerSecond.of(53.489))))
        .withName("Aim At Target Direct");
  }

  private static ShooterSetpoint createDefaultSetpoint() {
    ShooterSetpoint s = new ShooterSetpoint();
    s.robotYaw = Math.toRadians(180.0);
    s.hoodPitch = Math.toRadians(30.0);
    s.isValid = false;
    return s;
  }

  private static void updateSetpoint(
      RobotContainer container,
      Supplier<Translation3d> targetSupplier,
      ShooterSetpoint[] lastValidSetpoint) {

    var robotPose = RobotState.getInstance().getRobotPose();
    var robotSpeeds = RobotState.getInstance().getFieldRelativeSpeeds();
    var target = targetSupplier.get();

    ShooterSetpoint setpoint =
        GenericShooterResolver.resolve(robotPose, robotSpeeds, target, Constants.SHOOTER_CONFIG);

    // Log resolver outputs
    Logger.recordOutput("Aiming/SetpointValid", setpoint.isValid);
    Logger.recordOutput("Aiming/ActualTarget", new Pose3d(target, new Rotation3d()));

    if (setpoint.isValid) {
      lastValidSetpoint[0] = setpoint;

      // Log debug info
      Logger.recordOutput("Aiming/TurretYawDeg", Math.toDegrees(setpoint.robotYaw));
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
