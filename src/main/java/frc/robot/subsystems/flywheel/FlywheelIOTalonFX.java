package frc.robot.subsystems.flywheel;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;

public class FlywheelIOTalonFX implements FlywheelIO {

  private final TalonFX motor0, motor1, motor2;
  private final VelocityTorqueCurrentFOC velocityControl = new VelocityTorqueCurrentFOC(0);

  private final StatusSignal<AngularVelocity> velocity0, velocity1, velocity2;
  private final StatusSignal<Voltage> volts0, volts1, volts2;
  private final StatusSignal<Current> currentStator0, currentStator1, currentStator2;
  private final StatusSignal<Current> currentSupply0, currentSupply1, currentSupply2;

  public FlywheelIOTalonFX() {
    motor0 = new TalonFX(FlywheelConstants.kFlywheelMotor0CanID, Constants.kCANBus);
    motor1 = new TalonFX(FlywheelConstants.kFlywheelMotor1CanID, Constants.kCANBus);
    motor2 = new TalonFX(FlywheelConstants.kFlywheelMotor2CanID, Constants.kCANBus);

    velocity0 = motor0.getVelocity();
    volts0 = motor0.getMotorVoltage();
    currentStator0 = motor0.getStatorCurrent();
    currentSupply0 = motor0.getSupplyCurrent();

    velocity1 = motor1.getVelocity();
    volts1 = motor1.getMotorVoltage();
    currentStator1 = motor1.getStatorCurrent();
    currentSupply1 = motor1.getSupplyCurrent();

    velocity2 = motor2.getVelocity();
    volts2 = motor2.getMotorVoltage();
    currentStator2 = motor2.getStatorCurrent();
    currentSupply2 = motor2.getSupplyCurrent();

    var config = FlywheelConstants.getTalonFXConfig();
    motor0.getConfigurator().apply(config);
    motor1.getConfigurator().apply(config);
    motor2.getConfigurator().apply(config);

    BaseStatusSignal.setUpdateFrequencyForAll(
        50,
        velocity0,
        volts0,
        currentStator0,
        currentSupply0,
        velocity1,
        volts1,
        currentStator1,
        currentSupply1,
        velocity2,
        volts2,
        currentStator2,
        currentSupply2);

    motor0.optimizeBusUtilization();
    motor1.optimizeBusUtilization();
    motor2.optimizeBusUtilization();
  }

  @Override
  public void readInputs(FlywheelInputs inputs) {
    BaseStatusSignal.refreshAll(
        velocity0,
        volts0,
        currentStator0,
        currentSupply0,
        velocity1,
        volts1,
        currentStator1,
        currentSupply1,
        velocity2,
        volts2,
        currentStator2,
        currentSupply2);

    inputs.velocityRps0 = velocity0.getValueAsDouble();
    inputs.appliedVolts0 = volts0.getValueAsDouble();
    inputs.currentStatorAmps0 = currentStator0.getValueAsDouble();
    inputs.currentSupplyAmps0 = currentSupply0.getValueAsDouble();

    inputs.velocityRps1 = velocity1.getValueAsDouble();
    inputs.appliedVolts1 = volts1.getValueAsDouble();
    inputs.currentStatorAmps1 = currentStator1.getValueAsDouble();
    inputs.currentSupplyAmps1 = currentSupply1.getValueAsDouble();

    inputs.velocityRps2 = velocity2.getValueAsDouble();
    inputs.appliedVolts2 = volts2.getValueAsDouble();
    inputs.currentStatorAmps2 = currentStator2.getValueAsDouble();
    inputs.currentSupplyAmps2 = currentSupply2.getValueAsDouble();
  }

  @Override
  public void setVelocity(double rps0, double rps1, double rps2) {
    motor0.setControl(velocityControl.withVelocity(rps0));
    motor1.setControl(velocityControl.withVelocity(rps1));
    motor2.setControl(velocityControl.withVelocity(rps2));
  }
}
