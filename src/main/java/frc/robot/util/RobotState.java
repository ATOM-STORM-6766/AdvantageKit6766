package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import java.util.function.Supplier;

public class RobotState {
  private static final Translation3d ROBOT_TO_TURRET = new Translation3d(0.2, 0.0, 0.3);
  private static final Translation3d TURRET_TO_HOOD = new Translation3d(0.0, 0.0, 0.0);
  private static final Translation3d TURRET_TO_FLYWHEEL = new Translation3d(0.0, 0.0, 0.0);

  private final Supplier<Pose2d> robotPoseSupplier;
  private Rotation2d latestHoodRotation = new Rotation2d();
  private double latestTurretPositionRad = 0.0;
  private double latestTurretVelocityRadPerSec = 0.0;

  public RobotState() {
    this(() -> new Pose2d());
  }

  public RobotState(Supplier<Pose2d> robotPoseSupplier) {
    this.robotPoseSupplier = robotPoseSupplier;
  }

  public void addHoodRotation(Rotation2d rotation) {
    this.latestHoodRotation = rotation;
  }

  public void addTurretUpdates(
      double timestamp, Rotation2d absoluteRotation, double positionRad, double velocityRadPerSec) {
    this.latestTurretPositionRad = positionRad;
    this.latestTurretVelocityRadPerSec = velocityRadPerSec;
  }

  public Pose3d getTurretWorldPose() {
    Pose2d robotPose2d = robotPoseSupplier.get();
    Pose3d robotPose3d = new Pose3d(robotPose2d);
    Rotation3d turretYaw = new Rotation3d(0.0, 0.0, latestTurretPositionRad);
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
}
