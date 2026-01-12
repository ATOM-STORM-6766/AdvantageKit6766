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
import static frc.robot.util.PhoenixUtil.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.generated.TunerConstants;

/** IO implementation for real Turret hardware using TalonFX. Limited rotation range. */
public class TurretIOTalonFX implements TurretIO {
  private final TalonFX turretTalon;

  // Control requests
  private final VoltageOut voltageRequest = new VoltageOut(0);
  private final PositionVoltage positionVoltageRequest = new PositionVoltage(0.0);

  // Status signals
  private final StatusSignal<Angle> position;
  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<Voltage> appliedVolts;
  private final StatusSignal<Current> current;

  // Connection debouncer
  private final Debouncer connectedDebounce = new Debouncer(0.5);

  // Limit detection state
  private boolean resetting = false;
  private double lastPosition = 0.0;

  public TurretIOTalonFX() {
    turretTalon = new TalonFX(TURRET_MOTOR_ID, TunerConstants.kCANBus);

    // Configure motor
    var config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.Slot0.kP = TURRET_KP;
    config.Slot0.kI = TURRET_KI;
    config.Slot0.kD = TURRET_KD;
    config.Slot0.kS = TURRET_KS;
    config.Slot0.kV = TURRET_KV;
    config.Feedback.SensorToMechanismRatio = TURRET_GEAR_RATIO;
    config.MotionMagic.MotionMagicCruiseVelocity = TURRET_CRUISE_VELOCITY;
    config.MotionMagic.MotionMagicAcceleration = TURRET_ACCELERATION;
    config.ClosedLoopGeneral.ContinuousWrap = false; // Limited rotation range, no continuous wrap
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    tryUntilOk(5, () -> turretTalon.getConfigurator().apply(config, 0.25));
    tryUntilOk(5, () -> turretTalon.setPosition(0.0, 0.25));

    // Create status signals
    position = turretTalon.getPosition();
    velocity = turretTalon.getVelocity();
    appliedVolts = turretTalon.getMotorVoltage();
    current = turretTalon.getStatorCurrent();

    // Configure update frequencies
    BaseStatusSignal.setUpdateFrequencyForAll(50.0, position, velocity, appliedVolts, current);
    turretTalon.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(TurretIOInputs inputs) {
    // Refresh all signals
    var status = BaseStatusSignal.refreshAll(position, velocity, appliedVolts, current);

    // Update connection status
    inputs.connected = connectedDebounce.calculate(status.isOK());

    // Update position and velocity
    inputs.positionRad = Units.rotationsToRadians(position.getValueAsDouble());
    inputs.velocityRadPerSec = Units.rotationsToRadians(velocity.getValueAsDouble());
    inputs.appliedVolts = appliedVolts.getValueAsDouble();
    inputs.currentAmps = current.getValueAsDouble();

    // Limit detection based on stall current
    double currentAmps = inputs.currentAmps;
    double velocityRadPerSec = Math.abs(inputs.velocityRadPerSec);
    double positionRad = inputs.positionRad;
    double minAngle = MIN_TURRET_ANGLE.getRadians();
    double maxAngle = MAX_TURRET_ANGLE.getRadians();

    // Check for stall condition
    boolean isStalling =
        currentAmps > STALL_CURRENT_THRESHOLD && velocityRadPerSec < STALL_VELOCITY_THRESHOLD;

    // Determine limit direction based on movement
    if (isStalling) {
      double positionDelta = positionRad - lastPosition;
      // No wrap-around handling needed for limited rotation range

      if (Math.abs(positionDelta) < 0.001) {
        // Not moving, check which direction we're trying to move
        if (inputs.appliedVolts > 0.1) {
          inputs.atForwardLimit = true;
        } else if (inputs.appliedVolts < -0.1) {
          inputs.atReverseLimit = true;
        }
      } else if (positionDelta > 0) {
        // Moving forward, hit forward limit
        inputs.atForwardLimit = true;
        inputs.atReverseLimit = false;
      } else {
        // Moving backward, hit reverse limit
        inputs.atReverseLimit = true;
        inputs.atForwardLimit = false;
      }
    } else {
      // Not stalling, clear limit flags
      inputs.atForwardLimit = false;
      inputs.atReverseLimit = false;
    }

    // Also check if position is at physical limits (safety check)
    if (positionRad <= minAngle + 0.01) {
      inputs.atReverseLimit = true;
    }
    if (positionRad >= maxAngle - 0.01) {
      inputs.atForwardLimit = true;
    }

    lastPosition = positionRad;

    // Handle reset logic
    if (resetting) {
      if (inputs.atReverseLimit || positionRad <= minAngle + 0.01) {
        // Reached limit, set position to known limit
        tryUntilOk(5, () -> turretTalon.setPosition(Units.radiansToRotations(minAngle), 0.25));
        resetting = false;
        turretTalon.setControl(voltageRequest.withOutput(0.0));
      }
    }
  }

  @Override
  public void setPosition(Rotation2d angle) {
    resetting = false;
    // Limited rotation range - clamp target to physical limits
    double targetRad = angle.getRadians();
    double minAngle = MIN_TURRET_ANGLE.getRadians();
    double maxAngle = MAX_TURRET_ANGLE.getRadians();

    // Clamp target to the physical rotation range
    double clampedTarget = MathUtil.clamp(targetRad, minAngle, maxAngle);

    turretTalon.setControl(
        positionVoltageRequest.withPosition(Units.radiansToRotations(clampedTarget)));
  }

  @Override
  public void setOpenLoop(double output) {
    resetting = false;
    turretTalon.setControl(voltageRequest.withOutput(output * 12.0));
  }

  @Override
  public Rotation2d getPosition() {
    return Rotation2d.fromRadians(Units.rotationsToRadians(position.getValueAsDouble()));
  }

  @Override
  public void resetToLimit() {
    resetting = true;
    // Move towards reverse limit (minimum angle) slowly
    turretTalon.setControl(
        voltageRequest.withOutput(
            -Math.signum(RESET_SPEED_RAD_PER_SEC)
                * 12.0
                * Math.abs(RESET_SPEED_RAD_PER_SEC)
                / 10.0));
  }
}
