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
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.generated.TunerConstants;

/** IO implementation for real Hood hardware using TalonFX. */
public class HoodIOTalonFX implements HoodIO {
  private final TalonFX hoodTalon;

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

  public HoodIOTalonFX() {
    hoodTalon = new TalonFX(HOOD_MOTOR_ID, TunerConstants.kCANBus);

    // Configure motor
    var config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.Slot0.kP = HOOD_KP;
    config.Slot0.kI = HOOD_KI;
    config.Slot0.kD = HOOD_KD;
    config.Slot0.kS = HOOD_KS;
    config.Slot0.kV = HOOD_KV;
    config.Feedback.SensorToMechanismRatio = HOOD_GEAR_RATIO;
    config.MotionMagic.MotionMagicCruiseVelocity = HOOD_CRUISE_VELOCITY;
    config.MotionMagic.MotionMagicAcceleration = HOOD_ACCELERATION;
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    tryUntilOk(5, () -> hoodTalon.getConfigurator().apply(config, 0.25));
    tryUntilOk(5, () -> hoodTalon.setPosition(0.0, 0.25));

    // Create status signals
    position = hoodTalon.getPosition();
    velocity = hoodTalon.getVelocity();
    appliedVolts = hoodTalon.getMotorVoltage();
    current = hoodTalon.getStatorCurrent();

    // Configure update frequencies
    BaseStatusSignal.setUpdateFrequencyForAll(50.0, position, velocity, appliedVolts, current);
    hoodTalon.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(HoodIOInputs inputs) {
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

    // Check for stall condition
    boolean isStalling =
        currentAmps > STALL_CURRENT_THRESHOLD && velocityRadPerSec < STALL_VELOCITY_THRESHOLD;

    // Determine limit direction based on movement
    if (isStalling) {
      double positionDelta = positionRad - lastPosition;
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

    lastPosition = positionRad;

    // Handle reset logic
    if (resetting) {
      if (inputs.atReverseLimit) {
        // Reached limit, set position to known limit
        tryUntilOk(
            5,
            () ->
                hoodTalon.setPosition(Units.radiansToRotations(MIN_HOOD_ANGLE.getRadians()), 0.25));
        resetting = false;
        hoodTalon.setControl(voltageRequest.withOutput(0.0));
      }
    }
  }

  @Override
  public void setPosition(Rotation2d angle) {
    resetting = false;
    double targetRad = angle.getRadians();
    // Clamp to limits
    targetRad =
        Math.max(MIN_HOOD_ANGLE.getRadians(), Math.min(MAX_HOOD_ANGLE.getRadians(), targetRad));
    hoodTalon.setControl(positionVoltageRequest.withPosition(Units.radiansToRotations(targetRad)));
  }

  @Override
  public void setOpenLoop(double output) {
    resetting = false;
    hoodTalon.setControl(voltageRequest.withOutput(output * 12.0));
  }

  @Override
  public Rotation2d getPosition() {
    return Rotation2d.fromRadians(Units.rotationsToRadians(position.getValueAsDouble()));
  }

  @Override
  public void resetToLimit() {
    resetting = true;
    // Move towards reverse limit (minimum angle) slowly
    hoodTalon.setControl(
        voltageRequest.withOutput(
            -Math.signum(RESET_SPEED_RAD_PER_SEC)
                * 12.0
                * Math.abs(RESET_SPEED_RAD_PER_SEC)
                / 10.0));
  }
}
