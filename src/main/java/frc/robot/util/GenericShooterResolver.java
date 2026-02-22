package frc.robot.util;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.RobotState;

public class GenericShooterResolver {

  public static class ShooterConfig {
    public InterpolatingDoubleTreeMap flywheelRpsMap = new InterpolatingDoubleTreeMap();
    public InterpolatingTreeMap<Double, Rotation2d> hoodPitchRadiansMap =
        new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), Rotation2d::interpolate);
    public InterpolatingDoubleTreeMap timeOfFlightSecondsMap = new InterpolatingDoubleTreeMap();

    public Translation3d robotCenterToTurret = new Translation3d();

    public double minRange = 0.0;
    public double maxRange = 999.0;

    public double minHoodPitchDegrees = 0.0;
    public double maxHoodPitchDegrees = 0.0;

    public double maxFlywheelRps = 0.0;

    public int lookaheadIterations = 1;
    public double lookaheadDistanceEpsilonMeters = 1e-3;

    public double loopPeriodSecs = 0.02;

    private final LinearFilter driveAngleFilter =
        LinearFilter.movingAverage((int) (0.8 / loopPeriodSecs));
    private Rotation2d lastDriveAngle = null;
  }

  public static class ShooterSetpoint {
    public Angle robotYaw;
    public AngularVelocity robotYawRate;
    public Angle hoodPitch;
    public AngularVelocity flywheelRps;
    public double timeOfFlightSeconds;
    public boolean isValid;
    public Translation3d virtualTarget;

    public static ShooterSetpoint invalid() {
      ShooterSetpoint s = new ShooterSetpoint();
      s.isValid = false;
      s.robotYaw = Degrees.of(0.0);
      s.hoodPitch = Degrees.of(0.0);
      s.robotYawRate = RadiansPerSecond.of(0.0);
      s.flywheelRps = RotationsPerSecond.of(0.0);
      return s;
    }
  }

  public static ShooterSetpoint resolve(
      Pose2d robotPose,
      ChassisSpeeds robotFieldSpeeds,
      Translation3d targetPosition,
      ShooterConfig config) {

    ShooterSetpoint result = new ShooterSetpoint();

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
      return ShooterSetpoint.invalid();
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

    Translation2d lookaheadRobotCenter = finalLookaheadRobotPose.getTranslation();

    Translation2d virtualTargetXY =
        targetXY.minus(turretFieldVelXY.times(result.timeOfFlightSeconds));
    result.virtualTarget =
        new Translation3d(virtualTargetXY.getX(), virtualTargetXY.getY(), targetPosition.getZ());

    Translation2d yawTargetVector = targetXY.minus(lookaheadRobotCenter);

    Rotation2d driveAngle = yawTargetVector.getAngle();
    if (config.lastDriveAngle == null) config.lastDriveAngle = driveAngle;
    double filteredDriveAngleRate =
        config.driveAngleFilter.calculate(
            driveAngle.minus(config.lastDriveAngle).getRadians() / config.loopPeriodSecs);
    config.lastDriveAngle = driveAngle;

    var robotYaw = Radians.of(driveAngle.getRadians());
    var robotYawRate = RadiansPerSecond.of(filteredDriveAngleRate);
    var hoodPitch = config.hoodPitchRadiansMap.get(distanceMeters).getMeasure();
    var flywheelRps = RotationsPerSecond.of(config.flywheelRpsMap.get(distanceMeters));

    if (hoodPitch.in(Degrees) < config.minHoodPitchDegrees
        || hoodPitch.in(Degrees) > config.maxHoodPitchDegrees) {
      return ShooterSetpoint.invalid();
    }

    if (flywheelRps.in(RotationsPerSecond) > config.maxFlywheelRps) {
      return ShooterSetpoint.invalid();
    }

    // 如果虚拟目标点出现在车后面，返回无效
    if (AllianceFlipUtil.shouldFlip()
        ? (virtualTargetXY.getX() > RobotState.getInstance().getRobotPose().getX())
        : (virtualTargetXY.getX() < RobotState.getInstance().getRobotPose().getX())) {
      return ShooterSetpoint.invalid();
    }

    result.robotYaw = robotYaw;
    result.robotYawRate = robotYawRate;
    result.hoodPitch = hoodPitch;
    result.flywheelRps = flywheelRps;
    result.isValid = true;

    return result;
  }
}
