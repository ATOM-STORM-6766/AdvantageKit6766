package frc.robot.commands;

import static frc.robot.Constants.DriveControlConstants.FollowPointConstants.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.Drive;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class FollowPoint extends Command {

  private static final double LOOP_PERIOD_SECS = 0.02;

  private final Drive drive;
  private final Supplier<Pose2d> target;

  private final TrapezoidProfile driveProfile =
      new TrapezoidProfile(
          new TrapezoidProfile.Constraints(DRIVE_MAX_VELOCITY, DRIVE_MAX_ACCELERATION));

  private final PIDController driveController =
      new PIDController(DRIVE_KP, 0.0, DRIVE_KD, LOOP_PERIOD_SECS);
  private final ProfiledPIDController thetaController =
      new ProfiledPIDController(
          THETA_KP,
          0.0,
          THETA_KD,
          new TrapezoidProfile.Constraints(THETA_MAX_VELOCITY, THETA_MAX_ACCELERATION),
          LOOP_PERIOD_SECS);

  private Translation2d lastSetpointTranslation = Translation2d.kZero;
  private Translation2d lastSetpointVelocity = Translation2d.kZero;
  private double driveErrorAbs = 0.0;
  private double thetaErrorAbs = 0.0;
  private boolean running = false;

  public FollowPoint(Drive drive, Supplier<Pose2d> target) {
    this.drive = drive;
    this.target = target;
    addRequirements(drive);

    driveController.setTolerance(DRIVE_TOLERANCE);
    thetaController.setTolerance(THETA_TOLERANCE);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
  }

  public FollowPoint(Drive drive, Pose2d target) {
    this(drive, () -> target);
  }

  @Override
  public void initialize() {
    Pose2d currentPose = drive.getPose();
    ChassisSpeeds fieldVelocity = drive.getFieldRelativeChassisSpeeds();

    driveController.reset();
    thetaController.reset(
        currentPose.getRotation().getRadians(), fieldVelocity.omegaRadiansPerSecond);
    lastSetpointTranslation = currentPose.getTranslation();

    // 裁剪初始速度至规划器上限，防止轨迹末速超限时 TrapezoidProfile 收到不可行初始状态
    Translation2d rawVelocity =
        new Translation2d(fieldVelocity.vxMetersPerSecond, fieldVelocity.vyMetersPerSecond);
    double speed = rawVelocity.getNorm();
    lastSetpointVelocity =
        speed > DRIVE_MAX_VELOCITY ? rawVelocity.times(DRIVE_MAX_VELOCITY / speed) : rawVelocity;
  }

  @Override
  public void execute() {
    running = true;

    Pose2d currentPose = RobotState.getInstance().getRobotPose();
    Pose2d targetPose = target.get();

    Pose2d poseError = currentPose.relativeTo(targetPose);
    driveErrorAbs = poseError.getTranslation().getNorm();
    thetaErrorAbs = Math.abs(poseError.getRotation().getRadians());

    // ── 平移控制 ──────────────────────────────────────────────
    var direction = targetPose.getTranslation().minus(lastSetpointTranslation).toVector();
    double setpointVelocity =
        direction.norm() <= MIN_DISTANCE_VELOCITY_CORRECTION
            ? lastSetpointVelocity.getNorm()
            : lastSetpointVelocity.toVector().dot(direction) / direction.norm();
    setpointVelocity = Math.max(setpointVelocity, SETPOINT_MIN_VELOCITY);

    State driveSetpoint =
        driveProfile.calculate(
            LOOP_PERIOD_SECS, new State(direction.norm(), -setpointVelocity), new State(0, 0));

    double driveVelocityScalar =
        driveController.calculate(driveErrorAbs, driveSetpoint.position) + driveSetpoint.velocity;
    if (driveErrorAbs < driveController.getErrorTolerance()) driveVelocityScalar = 0.0;

    Rotation2d targetToCurrentAngle =
        currentPose.getTranslation().minus(targetPose.getTranslation()).getAngle();
    Translation2d driveVelocity = new Translation2d(driveVelocityScalar, targetToCurrentAngle);
    lastSetpointTranslation =
        new Pose2d(targetPose.getTranslation(), targetToCurrentAngle)
            .transformBy(
                new Transform2d(new Translation2d(driveSetpoint.position, 0.0), Rotation2d.kZero))
            .getTranslation();
    lastSetpointVelocity = new Translation2d(driveSetpoint.velocity, targetToCurrentAngle);

    // ── 旋转控制 ──────────────────────────────────────────────
    double thetaVelocity =
        thetaController.calculate(
                currentPose.getRotation().getRadians(),
                new State(targetPose.getRotation().getRadians(), 0.0))
            + thetaController.getSetpoint().velocity;
    if (thetaErrorAbs < thetaController.getPositionTolerance()) thetaVelocity = 0.0;

    // ── 输出底盘速度 ──────────────────────────────────────────
    drive.runVelocity(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            driveVelocity.getX(), driveVelocity.getY(), thetaVelocity, currentPose.getRotation()));

    Logger.recordOutput("FollowPoint/DistanceMeasured", driveErrorAbs);
    Logger.recordOutput("FollowPoint/DistanceSetpoint", driveSetpoint.position);
    Logger.recordOutput("FollowPoint/ThetaMeasured", currentPose.getRotation().getRadians());
    Logger.recordOutput("FollowPoint/ThetaSetpoint", thetaController.getSetpoint().position);
    Logger.recordOutput("FollowPoint/Goal", new Pose2d[] {targetPose});
    Logger.recordOutput(
        "FollowPoint/Setpoint",
        new Pose2d[] {
          new Pose2d(
              lastSetpointTranslation,
              Rotation2d.fromRadians(thetaController.getSetpoint().position))
        });
  }

  @Override
  public boolean isFinished() {
    return withinTolerance(DRIVE_TOLERANCE, Rotation2d.fromRadians(THETA_TOLERANCE));
  }

  @Override
  public void end(boolean interrupted) {
    drive.stop();
    running = false;

    Logger.recordOutput("FollowPoint/Goal", new Pose2d[] {});
    Logger.recordOutput("FollowPoint/Setpoint", new Pose2d[] {});
  }

  /** 当前是否在指定容差范围内 */
  public boolean withinTolerance(double driveTolerance, Rotation2d thetaTolerance) {
    return running && driveErrorAbs < driveTolerance && thetaErrorAbs < thetaTolerance.getRadians();
  }
}
