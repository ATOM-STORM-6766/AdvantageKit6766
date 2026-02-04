package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class RobotState {
  private static final Translation3d ROBOT_TO_TURRET = new Translation3d(0.2, 0.0, 0.3);
  private static final Translation3d TURRET_TO_HOOD = new Translation3d(0.0, 0.0, 0.0);
  private static final Translation3d TURRET_TO_FLYWHEEL = new Translation3d(0.0, 0.0, 0.0);

  private static final RobotState instance = new RobotState();

  private volatile Pose2d robotPose = new Pose2d();
  private volatile ChassisSpeeds robotRelativeSpeeds = new ChassisSpeeds();
  private volatile ChassisSpeeds fieldRelativeSpeeds = new ChassisSpeeds();
  private volatile Rotation2d latestHoodRotation = new Rotation2d();
  private volatile Rotation2d latestTurretRotation = new Rotation2d();

  private RobotState() {}

  public static RobotState getInstance() {
    return instance;
  }

  public void addRobotPose(Pose2d pose) {
    robotPose = pose;
  }

  public void addRobotRelativeSpeeds(ChassisSpeeds speeds) {
    robotRelativeSpeeds = copyChassisSpeeds(speeds);
  }

  public void addFieldRelativeSpeeds(ChassisSpeeds speeds) {
    fieldRelativeSpeeds = copyChassisSpeeds(speeds);
  }

  public Pose2d getRobotPose() {
    return robotPose;
  }

  public Pose2d getRobotPose(double lookaheadSeconds) {
    Pose2d current = getRobotPose();
    ChassisSpeeds field = getFieldRelativeSpeeds();
    Translation2d deltaTranslation =
        new Translation2d(
            field.vxMetersPerSecond * lookaheadSeconds,
            field.vyMetersPerSecond * lookaheadSeconds);
    Rotation2d deltaRotation =
        new Rotation2d(field.omegaRadiansPerSecond * lookaheadSeconds);
    return new Pose2d(
        current.getTranslation().plus(deltaTranslation),
        current.getRotation().plus(deltaRotation));
  }

  public Rotation2d getRobotRotation() {
    return getRobotPose().getRotation();
  }

  public ChassisSpeeds getRobotRelativeSpeeds() {
    return copyChassisSpeeds(robotRelativeSpeeds);
  }

  public ChassisSpeeds getFieldRelativeSpeeds() {
    return copyChassisSpeeds(fieldRelativeSpeeds);
  }

  public void addHoodRotation(Rotation2d rotation) {
    latestHoodRotation = rotation;
  }

  public void addTurretRotation(Rotation2d rotation) {
    latestTurretRotation = rotation;
  }

  public Pose3d getTurretWorldPose() {
    Pose2d robotPose2d = getRobotPose();
    Pose3d robotPose3d = new Pose3d(robotPose2d);
    Rotation3d turretYaw = new Rotation3d(0.0, 0.0, latestTurretRotation.getRadians());
    return robotPose3d.transformBy(new Transform3d(ROBOT_TO_TURRET, turretYaw));
  }

  public Pose3d getHoodWorldPose() {
    Rotation3d hoodPitch = new Rotation3d(0.0, latestHoodRotation.getRadians(), 0.0);
    return getTurretWorldPose().transformBy(new Transform3d(TURRET_TO_HOOD, hoodPitch));
  }

  public Pose3d getFlywheelWorldPose() {
    Rotation3d hoodPitch = new Rotation3d(0.0, latestHoodRotation.getRadians(), 0.0);
    return getTurretWorldPose().transformBy(new Transform3d(TURRET_TO_FLYWHEEL, hoodPitch));
  }

  private static ChassisSpeeds copyChassisSpeeds(ChassisSpeeds speeds) {
    return new ChassisSpeeds(
        speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond);
  }
}
