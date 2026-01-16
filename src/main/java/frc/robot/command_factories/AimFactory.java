package frc.robot.command_factories;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.subsystems.hood.HoodConstants;
import frc.robot.subsystems.turret.TurretConstants;
import frc.robot.util.GenericShooterResolver.ShooterSetpoint;
import java.util.function.Supplier;

public class AimFactory {
  public static Command alignSuperstructure(
      RobotContainer container, Supplier<ShooterSetpoint> setpointSupplier) {
    return new ParallelCommandGroup(
            container
                .getHood()
                .positionSetpointCommand(
                    () -> setpointSupplier.get().hoodPitch,
                    () -> setpointSupplier.get().hoodFeedforward),
            container
                .getTurret()
                .positionSetpointCommand(
                    () -> setpointSupplier.get().turretYaw,
                    () -> setpointSupplier.get().turretFeedforward)
            //     container
            //         .getFlywheel()
            //         .velocitySetpointCommand(() -> setpointSupplier.get().launchSpeed)
            )
        .withName("Align Superstructure");
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

    return new ParallelCommandGroup(
            container
                .getTurret()
                .positionSetpointCommand(
                    () -> {
                      double t = Timer.getFPGATimestamp();
                      double target = turretMid + turretAmp * Math.sin(omega * t);
                      return MathUtil.clamp(target, turretMin, turretMax);
                    },
                    () -> 0.0),
            container
                .getHood()
                .positionSetpointCommand(
                    () -> {
                      double t = Timer.getFPGATimestamp();
                      double target = hoodMid + hoodAmp * Math.sin(omega * t);
                      return MathUtil.clamp(target, hoodMin, hoodMax);
                    },
                    () -> 0.0))
        .withName("Sweep Turret And Hood (teleop)");
  }
}
