package frc.robot.subsystems.flywheel;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.generated.TunerConstants;

public class FlywheelIOTalonFX implements FlywheelIO {

  private final TalonFX motor;
  private final VelocityVoltage velocityControl = new VelocityVoltage(0);
  private final VoltageOut voltageControl = new VoltageOut(0);

  private final StatusSignal<AngularVelocity> velocitySignal;
  private final StatusSignal<Voltage> voltsSignal;
  private final StatusSignal<Current> currentStatorSignal;
  private final StatusSignal<Current> currentSupplySignal;

  public FlywheelIOTalonFX() {
    motor = new TalonFX(FlywheelConstants.kFlywheelMotorCanID, TunerConstants.kCANBus);

    velocitySignal = motor.getVelocity();
    voltsSignal = motor.getMotorVoltage();
    currentStatorSignal = motor.getStatorCurrent();
    currentSupplySignal = motor.getSupplyCurrent();

    var config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    // PID + Feedforward configuration
    config.Slot0.kS = FlywheelConstants.kS;
    config.Slot0.kP = FlywheelConstants.kP;
    config.Slot0.kI = FlywheelConstants.kI;
    config.Slot0.kD = FlywheelConstants.kD;
    config.Slot0.kV = FlywheelConstants.kV;
    config.Slot0.kA = FlywheelConstants.kA;

    if (RobotBase.isReal()) {
      config.CurrentLimits.StatorCurrentLimit = FlywheelConstants.kStatorCurrentLimit;
      config.CurrentLimits.StatorCurrentLimitEnable = FlywheelConstants.kStatorCurrentLimitEnable;
      config.CurrentLimits.SupplyCurrentLimit = FlywheelConstants.kSupplyCurrentLimit;
      config.CurrentLimits.SupplyCurrentLimitEnable = FlywheelConstants.kSupplyCurrentLimitEnable;
      config.ClosedLoopRamps = FlywheelConstants.makeClosedLoopRampConfig();
      config.OpenLoopRamps = FlywheelConstants.makeOpenLoopRampConfig();
    }

    motor.getConfigurator().apply(config);

    BaseStatusSignal.setUpdateFrequencyForAll(
        50, velocitySignal, voltsSignal, currentStatorSignal, currentSupplySignal);
    motor.optimizeBusUtilization();
  }

  @Override
  public void readInputs(FlywheelInputs inputs) {
    BaseStatusSignal.refreshAll(
        velocitySignal, voltsSignal, currentStatorSignal, currentSupplySignal);

    inputs.velocityRPM = velocitySignal.getValueAsDouble() * 60.0; // Convert RPS to RPM
    inputs.appliedVolts = voltsSignal.getValueAsDouble();
    inputs.currentStatorAmps = currentStatorSignal.getValueAsDouble();
    inputs.currentSupplyAmps = currentSupplySignal.getValueAsDouble();
  }

  @Override
  public void setVelocity(double velocityRPM, double ffVolts) {
    double velocityRPS = velocityRPM / 60.0;
    motor.setControl(velocityControl.withVelocity(velocityRPS).withFeedForward(ffVolts));
  }

  @Override
  public void setVoltage(double volts) {
    motor.setControl(voltageControl.withOutput(volts));
  }

  @Override
  public void stop() {
    motor.setControl(voltageControl.withOutput(0));
  }
}
