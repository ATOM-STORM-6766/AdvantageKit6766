package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.util.GenericShooterResolver.ShooterSetpoint;

public class RobotState {
  private static final RobotState instance = new RobotState();

  private volatile Pose2d robotPose = new Pose2d();
  private volatile ChassisSpeeds fieldRelativeSpeeds = new ChassisSpeeds();
  private volatile ShooterSetpoint aimSetpoint = ShooterSetpoint.invalid();

  private RobotState() {}

  public static RobotState getInstance() {
    return instance;
  }

  public void addRobotPose(Pose2d pose) {
    robotPose = pose;
  }

  public void addFieldRelativeSpeeds(ChassisSpeeds speeds) {
    fieldRelativeSpeeds = speeds;
  }

  public Pose2d getRobotPose() {
    return robotPose;
  }

  public Pose2d getRobotPose(double lookaheadSeconds) {
    Pose2d current = getRobotPose();
    ChassisSpeeds field = getFieldRelativeSpeeds();
    Translation2d deltaTranslation =
        new Translation2d(
            field.vxMetersPerSecond * lookaheadSeconds, field.vyMetersPerSecond * lookaheadSeconds);
    // Rotation2d deltaRotation = new Rotation2d(field.omegaRadiansPerSecond * lookaheadSeconds);
    return new Pose2d(
        current.getTranslation().plus(deltaTranslation),
        current.getRotation()); // .plus(deltaRotation));
  }

  public ChassisSpeeds getFieldRelativeSpeeds() {
    return fieldRelativeSpeeds;
  }

  public void setAimSetpoint(ShooterSetpoint setpoint) {
    aimSetpoint = setpoint;
  }

  public ShooterSetpoint getAimSetpoint() {
    return aimSetpoint;
  }
}
