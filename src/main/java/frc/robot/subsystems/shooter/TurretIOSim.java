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

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

/**
 * Physics sim implementation of turret IO. Simulation is always based on voltage control. Supports
 * continuous rotation.
 */
public class TurretIOSim implements TurretIO {
  private static final DCMotor TURRET_GEARBOX = DCMotor.getKrakenX60Foc(1);
  private static final double TURRET_INERTIA = 0.01; // kg*m^2, TODO: Measure actual inertia

  private final DCMotorSim turretSim;
  private final PIDController positionController;

  private boolean closedLoop = false;
  private double appliedVolts = 0.0;
  private boolean resetting = false;

  public TurretIOSim() {
    turretSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(TURRET_GEARBOX, TURRET_INERTIA, TURRET_GEAR_RATIO),
            TURRET_GEARBOX);

    positionController = new PIDController(TURRET_KP, TURRET_KI, TURRET_KD);
    // Enable continuous input for turret (wraps around -180 to 180 degrees)
    positionController.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public void updateInputs(TurretIOInputs inputs) {
    // Run closed-loop control
    if (closedLoop && !resetting) {
      double positionRad = turretSim.getAngularPositionRad();
      double targetRad = positionController.getSetpoint();
      double ffVolts = TURRET_KS * Math.signum(targetRad - positionRad) + TURRET_KV * 0.0;
      appliedVolts = ffVolts + positionController.calculate(positionRad);
    } else {
      positionController.reset();
    }

    // Simulate limit detection
    double positionRad = turretSim.getAngularPositionRad();
    double minRad = MIN_TURRET_ANGLE.getRadians();
    double maxRad = MAX_TURRET_ANGLE.getRadians();

    // Normalize position to [-pi, pi] for limit checking
    double normalizedPos = MathUtil.inputModulus(positionRad, -Math.PI, Math.PI);

    // Check if at limits
    boolean atMinLimit = normalizedPos <= minRad;
    boolean atMaxLimit = normalizedPos >= maxRad;

    // Simulate stall current when at limits
    if (atMinLimit || atMaxLimit) {
      // Clamp position to limits (but allow continuous rotation in simulation)
      if (normalizedPos < minRad) {
        positionRad = minRad;
      } else if (normalizedPos > maxRad) {
        positionRad = maxRad;
      }

      // Simulate high current when trying to move past limit
      if (Math.abs(appliedVolts) > 0.1) {
        inputs.currentAmps = STALL_CURRENT_THRESHOLD + 5.0;
      } else {
        inputs.currentAmps = 0.0;
      }
    }

    // Update simulation state
    if (!atMinLimit && !atMaxLimit) {
      turretSim.setInputVoltage(MathUtil.clamp(appliedVolts, -12.0, 12.0));
    } else {
      // Prevent movement past limits
      turretSim.setInputVoltage(0.0);
    }
    turretSim.update(0.02);

    // Update inputs
    inputs.connected = true;
    inputs.positionRad = turretSim.getAngularPositionRad();
    inputs.velocityRadPerSec = turretSim.getAngularVelocityRadPerSec();
    inputs.appliedVolts = appliedVolts;

    // Update limit flags
    inputs.atReverseLimit = atMinLimit;
    inputs.atForwardLimit = atMaxLimit;

    // Update current if not already set by limit detection
    if (!atMinLimit && !atMaxLimit) {
      inputs.currentAmps = Math.abs(turretSim.getCurrentDrawAmps());
    }
  }

  @Override
  public void setPosition(Rotation2d angle) {
    closedLoop = true;
    resetting = false;
    // Use continuous rotation - find shortest path
    double targetRad = angle.getRadians();
    positionController.setSetpoint(targetRad);
  }

  @Override
  public void setOpenLoop(double output) {
    closedLoop = false;
    resetting = false;
    appliedVolts = output * 12.0;
  }

  @Override
  public Rotation2d getPosition() {
    return Rotation2d.fromRadians(turretSim.getAngularPositionRad());
  }

  @Override
  public void resetToLimit() {
    resetting = true;
    closedLoop = false;
    // Move towards reverse limit (minimum angle)
    double currentPos = turretSim.getAngularPositionRad();
    double normalizedPos = MathUtil.inputModulus(currentPos, -Math.PI, Math.PI);
    if (normalizedPos > MIN_TURRET_ANGLE.getRadians() + 0.01) {
      appliedVolts =
          -Math.signum(RESET_SPEED_RAD_PER_SEC) * 12.0 * Math.abs(RESET_SPEED_RAD_PER_SEC) / 10.0;
    } else {
      // At limit, set position to known limit
      turretSim.setState(MIN_TURRET_ANGLE.getRadians(), 0.0);
      appliedVolts = 0.0;
      resetting = false;
    }
  }
}
