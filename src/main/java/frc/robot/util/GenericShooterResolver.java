package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.RobotState;

public class GenericShooterResolver {

  public static class ShooterConfig {
    public InterpolatingDoubleTreeMap flywheelRpsMap = new InterpolatingDoubleTreeMap();
    public InterpolatingTreeMap<Double, Rotation2d> hoodPitchRadiansMap =
        new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), Rotation2d::interpolate);
    public InterpolatingDoubleTreeMap timeOfFlightSecondsMap = new InterpolatingDoubleTreeMap();

    public Translation3d robotCenterToTurret = new Translation3d();

    public double turretMinYawRadians = Double.NEGATIVE_INFINITY;
    public double turretMaxYawRadians = Double.POSITIVE_INFINITY;

    public double minRange = 0.0;
    public double maxRange = 999.0;

    public int lookaheadIterations = 1;
    public double lookaheadDistanceEpsilonMeters = 1e-3;
  }

  public static class ShooterSetpointV2 {
    public double turretYaw;
    public double turretFeedforward;
    public double hoodPitch;
    public double hoodFeedforward;
    public double flywheelRps;
    public double timeOfFlightSeconds;
    public boolean isValid;
    public Translation3d virtualTarget;
    public Translation3d actualTarget;

    public static ShooterSetpointV2 invalid() {
      ShooterSetpointV2 s = new ShooterSetpointV2();
      s.isValid = false;
      return s;
    }
  }

  public static ShooterSetpointV2 resolve(
      Pose2d robotPose,
      ChassisSpeeds robotFieldSpeeds,
      Translation3d targetPosition,
      ShooterConfig config) {

    ShooterSetpointV2 result = new ShooterSetpointV2();
    result.actualTarget = targetPosition;

    Translation2d turretOffsetXY =
        new Translation2d(config.robotCenterToTurret.getX(), config.robotCenterToTurret.getY());

    Translation2d rField = turretOffsetXY.rotateBy(robotPose.getRotation());
    Translation2d turretFieldPosXY = robotPose.getTranslation().plus(rField);

    Translation2d tangentialVel =
        new Translation2d(-rField.getY(), rField.getX())
            .times(robotFieldSpeeds.omegaRadiansPerSecond);

    Translation2d turretFieldVelXY =
        new Translation2d(robotFieldSpeeds.vxMetersPerSecond, robotFieldSpeeds.vyMetersPerSecond)
            .plus(tangentialVel);

    Translation2d targetXY = new Translation2d(targetPosition.getX(), targetPosition.getY());

    double distanceMeters = targetXY.getDistance(turretFieldPosXY);
    if (distanceMeters < config.minRange || distanceMeters > config.maxRange) {
      return ShooterSetpointV2.invalid();
    }

    Translation2d lookaheadPosXY = turretFieldPosXY;
    double lastDistanceMeters = distanceMeters;
    int iterations = Math.max(0, config.lookaheadIterations);
    for (int i = 0; i < iterations; i++) {
      double tofSeconds = config.timeOfFlightSecondsMap.get(distanceMeters);
      Pose2d lookaheadRobotPose = RobotState.getInstance().getRobotPose(tofSeconds);
      lookaheadPosXY =
          lookaheadRobotPose
              .getTranslation()
              .plus(turretOffsetXY.rotateBy(lookaheadRobotPose.getRotation()));
      distanceMeters = targetXY.getDistance(lookaheadPosXY);
      if (Math.abs(distanceMeters - lastDistanceMeters) < config.lookaheadDistanceEpsilonMeters) {
        break;
      }
      lastDistanceMeters = distanceMeters;
    }

    result.timeOfFlightSeconds = config.timeOfFlightSecondsMap.get(distanceMeters);
    Pose2d finalLookaheadRobotPose =
        RobotState.getInstance().getRobotPose(result.timeOfFlightSeconds);
    lookaheadPosXY =
        finalLookaheadRobotPose
            .getTranslation()
            .plus(turretOffsetXY.rotateBy(finalLookaheadRobotPose.getRotation()));

    Translation2d aimVectorXY = targetXY.minus(lookaheadPosXY);
    Rotation2d turretYawField = aimVectorXY.getAngle();

    Translation2d virtualTargetXY =
        targetXY.minus(turretFieldVelXY.times(result.timeOfFlightSeconds));
    result.virtualTarget =
        new Translation3d(virtualTargetXY.getX(), virtualTargetXY.getY(), targetPosition.getZ());

    Rotation2d turretYawRobotRelative = turretYawField.minus(finalLookaheadRobotPose.getRotation());
    result.turretYaw = turretYawRobotRelative.getRadians();
    if (result.turretYaw < config.turretMinYawRadians
        || result.turretYaw > config.turretMaxYawRadians) {
      return ShooterSetpointV2.invalid();
    }

    result.hoodPitch = config.hoodPitchRadiansMap.get(distanceMeters).getRadians();
    result.flywheelRps = config.flywheelRpsMap.get(distanceMeters);

    result.turretFeedforward = 0.0;
    result.hoodFeedforward = 0.0;
    result.isValid = true;

    return result;
  }
}
