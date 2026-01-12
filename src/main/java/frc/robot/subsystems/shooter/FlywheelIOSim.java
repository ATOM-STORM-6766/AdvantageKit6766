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
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

/** Physics sim implementation of flywheel IO. Simulation is always based on voltage control. */
public class FlywheelIOSim implements FlywheelIO {
  private static final DCMotor FLYWHEEL_GEARBOX = DCMotor.getKrakenX60Foc(1);
  private static final double FLYWHEEL_INERTIA = 0.005; // kg*m^2, TODO: Measure actual inertia

  private final DCMotorSim flywheelSim;
  private final PIDController velocityController;

  private boolean closedLoop = false;
  private double appliedVolts = 0.0;

  public FlywheelIOSim() {
    flywheelSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                FLYWHEEL_GEARBOX, FLYWHEEL_INERTIA, FLYWHEEL_GEAR_RATIO),
            FLYWHEEL_GEARBOX);

    velocityController = new PIDController(FLYWHEEL_KP, FLYWHEEL_KI, FLYWHEEL_KD);
  }

  @Override
  public void updateInputs(FlywheelIOInputs inputs) {
    // Run closed-loop control
    if (closedLoop) {
      double currentVelocity = flywheelSim.getAngularVelocityRadPerSec();
      double targetVelocity = velocityController.getSetpoint();
      double ffVolts = FLYWHEEL_KS * Math.signum(targetVelocity) + FLYWHEEL_KV * targetVelocity;
      appliedVolts = ffVolts + velocityController.calculate(currentVelocity);
    } else {
      velocityController.reset();
    }

    // Update simulation state
    flywheelSim.setInputVoltage(MathUtil.clamp(appliedVolts, -12.0, 12.0));
    flywheelSim.update(0.02);

    // Update inputs
    inputs.connected = true;
    inputs.velocityRadPerSec = flywheelSim.getAngularVelocityRadPerSec();
    inputs.velocityRPM = Units.radiansPerSecondToRotationsPerMinute(inputs.velocityRadPerSec);
    inputs.appliedVolts = appliedVolts;
    inputs.currentAmps = Math.abs(flywheelSim.getCurrentDrawAmps());
  }

  @Override
  public void setVelocity(double velocityRadPerSec) {
    closedLoop = true;
    velocityController.setSetpoint(velocityRadPerSec);
  }

  @Override
  public void setRPM(double rpm) {
    setVelocity(Units.rotationsPerMinuteToRadiansPerSecond(rpm));
  }

  @Override
  public void setOpenLoop(double output) {
    closedLoop = false;
    appliedVolts = output * 12.0;
  }
}
