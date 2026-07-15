// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.
package frc.robot.commands;

import static frc.robot.Constants.DriveControlConstants.MoveToTrenchConstants.*;

import choreo.util.ChoreoAllianceFlipUtil;
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
import frc.robot.FieldConstants;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.AllianceFlipUtil;
import java.util.Optional;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class BlinkToTrench extends Command {

  private static final double LOOP_PERIOD_SECS = 0.02;

  private final Drive drive;
  private final Supplier<Pose2d> target;
  private final Supplier<Optional<Double>> omegaOverride;

  // 梯形速度规划器（约束固定，声明为字段避免重复构造）
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

  private BlinkToTrench(
      Drive drive, Supplier<Pose2d> target, Supplier<Optional<Double>> omegaOverride) {
    this.drive = drive;
    this.target = target;
    this.omegaOverride = omegaOverride;
    addRequirements(drive);

    driveController.setTolerance(DRIVE_TOLERANCE);
    thetaController.setTolerance(THETA_TOLERANCE);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
  }

  private BlinkToTrench(Drive drive, Supplier<Pose2d> target) {
    this(drive, target, () -> Optional.empty());
  }

  /** 前往 AprilTag 23（Trench）固定点 */
  public static Command createLeft(Drive drive) {
    return new BlinkToTrench(
            drive,
            () -> {
              var pose = drive.getPose();
              var blink = AllianceFlipUtil.apply(FieldConstants.Position.blink);
              return new Pose2d(pose.getX(), blink.getY(), pose.getRotation());
            })
        .andThen(
            new BlinkToTrench(drive, () -> AllianceFlipUtil.apply(FieldConstants.Position.blink)));
  }

  /** 前往 AprilTag 23（Trench）固定点 */
  public static Command createRight(Drive drive) {
    //  Rotation2d.fromRotations(     // ❌ 旋转被强制吸附到最近的 0° 或 180°
    //         Math.round(drive.getPose().getRotation().getRotations() * 2.0) / 2.0));
    return new BlinkToTrench(
            drive,
            () -> {
              var pose = drive.getPose();
              var blink =
                  AllianceFlipUtil.apply(
                      ChoreoAllianceFlipUtil.getMirrorY().flip(FieldConstants.Position.blink));
              return new Pose2d(pose.getX(), blink.getY(), pose.getRotation());
            })
        .andThen(
            new BlinkToTrench(
                drive,
                () ->
                    AllianceFlipUtil.apply(
                        ChoreoAllianceFlipUtil.getMirrorY().flip(FieldConstants.Position.blink))));
  }

  @Override
  public void initialize() {
    Pose2d currentPose = drive.getPose();
    ChassisSpeeds fieldVelocity = drive.getFieldRelativeChassisSpeeds();

    driveController.reset();
    thetaController.reset(
        currentPose.getRotation().getRadians(), fieldVelocity.omegaRadiansPerSecond);
    lastSetpointTranslation = currentPose.getTranslation();

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

    // PID 纠偏 + 规划速度前馈（kV=1）
    double driveVelocityScalar =
        driveController.calculate(driveErrorAbs, driveSetpoint.position) + driveSetpoint.velocity;

    // 基于剩余距离的安全速度上限，防止长距离过冲
    // v_max = sqrt(2 * a_max * d)，确保能在剩余距离内刹停
    double maxSafeVelocity =
        Math.sqrt(2.0 * DRIVE_MAX_ACCELERATION * Math.max(driveErrorAbs, 1e-6));
    if (driveVelocityScalar > maxSafeVelocity) {
      driveVelocityScalar = maxSafeVelocity;
    } else if (driveVelocityScalar < -maxSafeVelocity) {
      driveVelocityScalar = -maxSafeVelocity;
    }

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
    // 目标为固定点，目标角速度为 0；ProfiledPID 内部已处理规划曲线
    double thetaVelocity =
        thetaController.calculate(
                currentPose.getRotation().getRadians(),
                new State(targetPose.getRotation().getRadians(), 0.0))
            + thetaController.getSetpoint().velocity; // 规划角速度前馈（kV=1）
    if (thetaErrorAbs < thetaController.getPositionTolerance()) thetaVelocity = 0.0;

    // ── 输出底盘速度 ──────────────────────────────────────────
    Optional<Double> overrideOmega = omegaOverride.get();
    drive.runVelocity(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            driveVelocity.getX(),
            driveVelocity.getY(),
            overrideOmega.orElse(thetaVelocity),
            currentPose.getRotation()));

    Logger.recordOutput("DriveToPose/DistanceMeasured", driveErrorAbs);
    Logger.recordOutput("DriveToPose/DistanceSetpoint", driveSetpoint.position);
    Logger.recordOutput("DriveToPose/DistanceSetpointVelocity", driveSetpoint.velocity);
    Logger.recordOutput("DriveToPose/ThetaMeasured", currentPose.getRotation().getRadians());
    Logger.recordOutput("DriveToPose/ThetaSetpoint", thetaController.getSetpoint().position);
    Logger.recordOutput(
        "DriveToPose/ThetaSetpointVelocity", thetaController.getSetpoint().velocity);
    Logger.recordOutput(
        "DriveToPose/Setpoint",
        new Pose2d[] {
          new Pose2d(
              lastSetpointTranslation,
              Rotation2d.fromRadians(thetaController.getSetpoint().position))
        });
    Logger.recordOutput("DriveToPose/Goal", new Pose2d[] {targetPose});
  }

  @Override
  public boolean isFinished() {
    return withinTolerance(DRIVE_TOLERANCE, Rotation2d.fromRadians(THETA_TOLERANCE));
  }

  @Override
  public void end(boolean interrupted) {
    drive.stop();
    running = false;

    Logger.recordOutput("DriveToPose/Setpoint", new Pose2d[] {});
    Logger.recordOutput("DriveToPose/Goal", new Pose2d[] {});
  }

  /** 当前是否在指定容差范围内 */
  public boolean withinTolerance(double driveTolerance, Rotation2d thetaTolerance) {
    return running && driveErrorAbs < driveTolerance && thetaErrorAbs < thetaTolerance.getRadians();
  }
}
