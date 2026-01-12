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
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.generated.TunerConstants;

/** IO implementation for real Flywheel hardware using TalonFX. */
public class FlywheelIOTalonFX implements FlywheelIO {
  private final TalonFX flywheelTalon;

  // Control requests
  private final VoltageOut voltageRequest = new VoltageOut(0);
  private final VelocityVoltage velocityVoltageRequest = new VelocityVoltage(0.0);

  // Status signals
  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<Voltage> appliedVolts;
  private final StatusSignal<Current> current;

  // Connection debouncer
  private final Debouncer connectedDebounce = new Debouncer(0.5);

  public FlywheelIOTalonFX() {
    flywheelTalon = new TalonFX(FLYWHEEL_MOTOR_ID, TunerConstants.kCANBus);

    // Configure motor
    var config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast; // Coast for flywheel
    config.Slot0.kP = FLYWHEEL_KP;
    config.Slot0.kI = FLYWHEEL_KI;
    config.Slot0.kD = FLYWHEEL_KD;
    config.Slot0.kS = FLYWHEEL_KS;
    config.Slot0.kV = FLYWHEEL_KV;
    config.Feedback.SensorToMechanismRatio = FLYWHEEL_GEAR_RATIO;
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    tryUntilOk(5, () -> flywheelTalon.getConfigurator().apply(config, 0.25));

    // Create status signals
    velocity = flywheelTalon.getVelocity();
    appliedVolts = flywheelTalon.getMotorVoltage();
    current = flywheelTalon.getStatorCurrent();

    // Configure update frequencies
    BaseStatusSignal.setUpdateFrequencyForAll(50.0, velocity, appliedVolts, current);
    flywheelTalon.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(FlywheelIOInputs inputs) {
    // Refresh all signals
    var status = BaseStatusSignal.refreshAll(velocity, appliedVolts, current);

    // Update connection status
    inputs.connected = connectedDebounce.calculate(status.isOK());

    // Update velocity
    inputs.velocityRadPerSec = Units.rotationsToRadians(velocity.getValueAsDouble());
    inputs.velocityRPM = Units.radiansPerSecondToRotationsPerMinute(inputs.velocityRadPerSec);
    inputs.appliedVolts = appliedVolts.getValueAsDouble();
    inputs.currentAmps = current.getValueAsDouble();
  }

  @Override
  public void setVelocity(double velocityRadPerSec) {
    double velocityRotPerSec = Units.radiansToRotations(velocityRadPerSec);
    flywheelTalon.setControl(velocityVoltageRequest.withVelocity(velocityRotPerSec));
  }

  @Override
  public void setRPM(double rpm) {
    setVelocity(Units.rotationsPerMinuteToRadiansPerSecond(rpm));
  }

  @Override
  public void setOpenLoop(double output) {
    flywheelTalon.setControl(voltageRequest.withOutput(output * 12.0));
  }
}
