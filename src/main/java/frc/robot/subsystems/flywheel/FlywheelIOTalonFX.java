package frc.robot.subsystems.flywheel;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.MotionMagicVelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;

public class FlywheelIOTalonFX implements FlywheelIO {

  private final TalonFX motor;
  private final MotionMagicVelocityTorqueCurrentFOC velocityControl =
      new MotionMagicVelocityTorqueCurrentFOC(0);

  private final StatusSignal<AngularVelocity> velocitySignal;
  private final StatusSignal<Voltage> voltsSignal;
  private final StatusSignal<Current> currentStatorSignal;
  private final StatusSignal<Current> currentSupplySignal;

  public FlywheelIOTalonFX() {
    motor = new TalonFX(FlywheelConstants.kFlywheelMotorCanID, Constants.kCANBus);

    velocitySignal = motor.getVelocity();
    voltsSignal = motor.getMotorVoltage();
    currentStatorSignal = motor.getStatorCurrent();
    currentSupplySignal = motor.getSupplyCurrent();

    motor.getConfigurator().apply(FlywheelConstants.getTalonFXConfig());

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
}
