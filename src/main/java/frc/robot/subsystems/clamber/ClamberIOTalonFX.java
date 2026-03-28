package frc.robot.subsystems.clamber;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;

public class ClamberIOTalonFX implements ClamberIO {
  protected final TalonFX motor;

  private final StatusSignal<Angle> position;
  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<Voltage> appliedVolts;
  private final StatusSignal<Current> supplyCurrent;
  private final StatusSignal<Current> torqueCurrent;

  private final VoltageOut voltageRequest = new VoltageOut(0).withEnableFOC(true);

  public ClamberIOTalonFX() {
    motor = new TalonFX(ClamberConstants.kClamberMotorId, Constants.kCANBus);

    var config = new TalonFXConfiguration();
    config.MotorOutput.Inverted = ClamberConstants.kInverted;
    config.MotorOutput.NeutralMode = ClamberConstants.kNeutralMode;
    config.Feedback.SensorToMechanismRatio = ClamberConstants.kGearRatio;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = ClamberConstants.kCurrentLimit;
    config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0.01;
    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = ClamberConstants.maxPosition;

    motor.getConfigurator().apply(config);

    position = motor.getPosition();
    velocity = motor.getVelocity();
    appliedVolts = motor.getMotorVoltage();
    supplyCurrent = motor.getSupplyCurrent();
    torqueCurrent = motor.getTorqueCurrent();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, position, velocity, appliedVolts, supplyCurrent, torqueCurrent);

    motor.optimizeBusUtilization();
    motor.setPosition(ClamberConstants.maxPosition);
  }

  @Override
  public void updateInputs(ClamberInputs inputs) {
    BaseStatusSignal.refreshAll(position, velocity, appliedVolts, supplyCurrent, torqueCurrent);

    inputs.position = position.getValue();
    inputs.velocity = velocity.getValue();
    inputs.appliedVolts = appliedVolts.getValue();
    inputs.supplyCurrent = supplyCurrent.getValue();
    inputs.torqueCurrent = torqueCurrent.getValue();
  }

  @Override
  public void setVoltage(Voltage volts) {
    motor.setControl(voltageRequest.withOutput(volts));
  }

  @Override
  public void stop() {
    motor.stopMotor();
  }
}
