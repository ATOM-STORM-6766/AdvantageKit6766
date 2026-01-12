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

/** Physics sim implementation of hood IO. Simulation is always based on voltage control. */
public class HoodIOSim implements HoodIO {
  private static final DCMotor HOOD_GEARBOX = DCMotor.getKrakenX60Foc(1);
  private static final double HOOD_INERTIA = 0.01; // kg*m^2, TODO: Measure actual inertia

  private final DCMotorSim hoodSim;
  private final PIDController positionController;

  private boolean closedLoop = false;
  private double appliedVolts = 0.0;
  private boolean resetting = false;

  public HoodIOSim() {
    hoodSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(HOOD_GEARBOX, HOOD_INERTIA, HOOD_GEAR_RATIO),
            HOOD_GEARBOX);

    positionController = new PIDController(HOOD_KP, HOOD_KI, HOOD_KD);
  }

  @Override
  public void updateInputs(HoodIOInputs inputs) {
    // Run closed-loop control
    if (closedLoop && !resetting) {
      double positionRad = hoodSim.getAngularPositionRad();
      double targetRad = positionController.getSetpoint();
      double ffVolts = HOOD_KS * Math.signum(targetRad - positionRad) + HOOD_KV * 0.0;
      appliedVolts = ffVolts + positionController.calculate(positionRad);
    } else {
      positionController.reset();
    }

    // Simulate limit detection
    double positionRad = hoodSim.getAngularPositionRad();
    double minRad = MIN_HOOD_ANGLE.getRadians();
    double maxRad = MAX_HOOD_ANGLE.getRadians();

    // Check if at limits
    boolean atMinLimit = positionRad <= minRad;
    boolean atMaxLimit = positionRad >= maxRad;

    // Simulate stall current when at limits
    if (atMinLimit || atMaxLimit) {
      // Clamp position to limits
      positionRad = MathUtil.clamp(positionRad, minRad, maxRad);
      hoodSim.setState(positionRad, 0.0);

      // Simulate high current when trying to move past limit
      if (Math.abs(appliedVolts) > 0.1) {
        inputs.currentAmps = STALL_CURRENT_THRESHOLD + 5.0;
      } else {
        inputs.currentAmps = 0.0;
      }
    }

    // Update simulation state
    if (!atMinLimit && !atMaxLimit) {
      hoodSim.setInputVoltage(MathUtil.clamp(appliedVolts, -12.0, 12.0));
    } else {
      // Prevent movement past limits
      hoodSim.setInputVoltage(0.0);
    }
    hoodSim.update(0.02);

    // Update inputs
    inputs.connected = true;
    inputs.positionRad = MathUtil.clamp(hoodSim.getAngularPositionRad(), minRad, maxRad);
    inputs.velocityRadPerSec = hoodSim.getAngularVelocityRadPerSec();
    inputs.appliedVolts = appliedVolts;

    // Update limit flags
    inputs.atReverseLimit = atMinLimit;
    inputs.atForwardLimit = atMaxLimit;

    // Update current if not already set by limit detection
    if (!atMinLimit && !atMaxLimit) {
      inputs.currentAmps = Math.abs(hoodSim.getCurrentDrawAmps());
    }
  }

  @Override
  public void setPosition(Rotation2d angle) {
    closedLoop = true;
    resetting = false;
    double targetRad =
        MathUtil.clamp(
            angle.getRadians(), MIN_HOOD_ANGLE.getRadians(), MAX_HOOD_ANGLE.getRadians());
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
    return Rotation2d.fromRadians(hoodSim.getAngularPositionRad());
  }

  @Override
  public void resetToLimit() {
    resetting = true;
    closedLoop = false;
    // Move towards reverse limit (minimum angle)
    double currentPos = hoodSim.getAngularPositionRad();
    if (currentPos > MIN_HOOD_ANGLE.getRadians() + 0.01) {
      appliedVolts =
          -Math.signum(RESET_SPEED_RAD_PER_SEC) * 12.0 * Math.abs(RESET_SPEED_RAD_PER_SEC) / 10.0;
    } else {
      // At limit, set position to known limit
      hoodSim.setState(MIN_HOOD_ANGLE.getRadians(), 0.0);
      appliedVolts = 0.0;
      resetting = false;
    }
  }
}
