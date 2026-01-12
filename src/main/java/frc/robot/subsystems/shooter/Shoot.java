// Copyright 2021-2025 FRC 6766
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.shooter;

import static frc.robot.subsystems.shooter.ShooterConstants.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

/** Shooter subsystem that coordinates Hood, Turret, and Flywheel components. */
public class Shoot extends SubsystemBase {
  private final HoodIO hoodIO;
  private final TurretIO turretIO;
  private final FlywheelIO flywheelIO;

  private final HoodIOInputsAutoLogged hoodInputs = new HoodIOInputsAutoLogged();
  private final TurretIOInputsAutoLogged turretInputs = new TurretIOInputsAutoLogged();
  private final FlywheelIOInputsAutoLogged flywheelInputs = new FlywheelIOInputsAutoLogged();

  private final Alert hoodDisconnectedAlert =
      new Alert("Hood motor is disconnected.", AlertType.kWarning);
  private final Alert turretDisconnectedAlert =
      new Alert("Turret motor is disconnected.", AlertType.kWarning);
  private final Alert flywheelDisconnectedAlert =
      new Alert("Flywheel motor is disconnected.", AlertType.kWarning);

  // Target setpoints
  private double targetFlywheelRPM = 0.0;
  private Rotation2d targetHoodAngle = new Rotation2d();
  private Rotation2d targetTurretAngle = new Rotation2d();

  public Shoot(HoodIO hoodIO, TurretIO turretIO, FlywheelIO flywheelIO) {
    this.hoodIO = hoodIO;
    this.turretIO = turretIO;
    this.flywheelIO = flywheelIO;
  }

  @Override
  public void periodic() {
    // Update all IO inputs
    hoodIO.updateInputs(hoodInputs);
    turretIO.updateInputs(turretInputs);
    flywheelIO.updateInputs(flywheelInputs);

    // Log inputs
    Logger.processInputs("Shooter/Hood", hoodInputs);
    Logger.processInputs("Shooter/Turret", turretInputs);
    Logger.processInputs("Shooter/Flywheel", flywheelInputs);

    // Update disconnected alerts
    hoodDisconnectedAlert.set(!hoodInputs.connected);
    turretDisconnectedAlert.set(!turretInputs.connected);
    flywheelDisconnectedAlert.set(!flywheelInputs.connected);

    // Log target setpoints
    Logger.recordOutput("Shooter/TargetFlywheelRPM", targetFlywheelRPM);
    Logger.recordOutput("Shooter/TargetHoodAngle", targetHoodAngle);
    Logger.recordOutput("Shooter/TargetTurretAngle", targetTurretAngle);
  }

  /**
   * Calculates and sets target values for aiming at a target position.
   *
   * @param robotPose Current robot pose
   * @param robotRotation Current robot rotation
   * @param robotVelocity Current robot velocity
   * @param targetPose Target position to aim at
   */
  public void aimAtTarget(
      Pose2d robotPose, Rotation2d robotRotation, ChassisSpeeds robotVelocity, Pose2d targetPose) {
    // TODO: Implement trajectory calculation algorithm
    // This is a placeholder that calculates basic angles
    // In a real implementation, this would use physics-based trajectory calculations

    // Calculate direction to target
    double dx = targetPose.getX() - robotPose.getX();
    double dy = targetPose.getY() - robotPose.getY();
    Rotation2d directionToTarget = new Rotation2d(dx, dy);

    // Calculate turret angle (relative to robot)
    Rotation2d turretAngle = directionToTarget.minus(robotRotation);

    // Calculate hood angle (placeholder - should be based on distance and physics)
    Rotation2d hoodAngle = Rotation2d.fromDegrees(30.0); // TODO: Calculate based on distance

    // Calculate flywheel RPM (placeholder - should be based on distance and physics)
    double flywheelRPM = 3000.0; // TODO: Calculate based on distance and target height

    // Set targets
    setTurretAngle(turretAngle);
    setHoodAngle(hoodAngle);
    setFlywheelRPM(flywheelRPM);
  }

  /**
   * Sets the target flywheel RPM.
   *
   * @param rpm Target RPM
   */
  public void setFlywheelRPM(double rpm) {
    targetFlywheelRPM = Math.max(0.0, Math.min(MAX_FLYWHEEL_RPM, rpm));
    flywheelIO.setRPM(targetFlywheelRPM);
  }

  /**
   * Sets the target hood angle.
   *
   * @param angle Target angle
   */
  public void setHoodAngle(Rotation2d angle) {
    // Clamp to limits
    double angleRad = angle.getRadians();
    angleRad =
        Math.max(MIN_HOOD_ANGLE.getRadians(), Math.min(MAX_HOOD_ANGLE.getRadians(), angleRad));
    targetHoodAngle = Rotation2d.fromRadians(angleRad);
    hoodIO.setPosition(targetHoodAngle);
  }

  /**
   * Sets the target turret angle.
   *
   * @param angle Target angle
   */
  public void setTurretAngle(Rotation2d angle) {
    targetTurretAngle = angle;
    turretIO.setPosition(targetTurretAngle);
  }

  /**
   * Checks if all components are ready (at target values within tolerance).
   *
   * @return True if ready to shoot
   */
  public boolean isReady() {
    // Check hood position
    Rotation2d hoodError = targetHoodAngle.minus(Rotation2d.fromRadians(hoodInputs.positionRad));
    boolean hoodReady =
        Math.abs(hoodError.getRadians()) < HOOD_POSITION_TOLERANCE_RAD
            && Math.abs(hoodInputs.velocityRadPerSec) < HOOD_VELOCITY_TOLERANCE_RAD_PER_SEC;

    // Check turret position
    Rotation2d turretError =
        targetTurretAngle.minus(Rotation2d.fromRadians(turretInputs.positionRad));
    boolean turretReady =
        Math.abs(turretError.getRadians()) < TURRET_POSITION_TOLERANCE_RAD
            && Math.abs(turretInputs.velocityRadPerSec) < TURRET_VELOCITY_TOLERANCE_RAD_PER_SEC;

    // Check flywheel velocity
    double flywheelError = Math.abs(targetFlywheelRPM - flywheelInputs.velocityRPM);
    boolean flywheelReady = flywheelError < FLYWHEEL_RPM_TOLERANCE;

    return hoodReady && turretReady && flywheelReady;
  }

  /** Stops all shooter components. */
  public void stop() {
    hoodIO.setOpenLoop(0.0);
    turretIO.setOpenLoop(0.0);
    flywheelIO.setOpenLoop(0.0);
    targetFlywheelRPM = 0.0;
    targetHoodAngle = new Rotation2d();
    targetTurretAngle = new Rotation2d();
  }

  /** Resets the hood to the limit position. */
  public void resetHood() {
    hoodIO.resetToLimit();
  }

  /** Resets the turret to the limit position. */
  public void resetTurret() {
    turretIO.resetToLimit();
  }

  /**
   * Gets the current hood position.
   *
   * @return Current hood angle
   */
  public Rotation2d getHoodPosition() {
    return hoodIO.getPosition();
  }

  /**
   * Gets the current turret position.
   *
   * @return Current turret angle
   */
  public Rotation2d getTurretPosition() {
    return turretIO.getPosition();
  }

  /**
   * Gets the current flywheel RPM.
   *
   * @return Current flywheel RPM
   */
  public double getFlywheelRPM() {
    return flywheelInputs.velocityRPM;
  }
}
