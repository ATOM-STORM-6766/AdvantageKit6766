package frc.robot.subsystems.flywheel;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;

public class FlywheelIOTalonFX implements FlywheelIO {

  protected final TalonFX motor0;
  protected final TalonFX motor1;
  protected final TalonFX motor2;
  protected final TalonFX motor3;

  private final StatusSignal<AngularVelocity> velocitySignal0;
  private final StatusSignal<Voltage> voltsSignal0;
  private final StatusSignal<Current> currentTorqueSignal0;

  private final StatusSignal<AngularVelocity> velocitySignal1;
  private final StatusSignal<Voltage> voltsSignal1;
  private final StatusSignal<Current> currentTorqueSignal1;

  private final StatusSignal<AngularVelocity> velocitySignal2;
  private final StatusSignal<Voltage> voltsSignal2;
  private final StatusSignal<Current> currentTorqueSignal2;

  private final StatusSignal<AngularVelocity> velocitySignal3;
  private final StatusSignal<Voltage> voltsSignal3;
  private final StatusSignal<Current> currentTorqueSignal3;

  private final VelocityTorqueCurrentFOC velocityControl =
      new VelocityTorqueCurrentFOC(0).withUseTimesync(true);

  public FlywheelIOTalonFX() {
    motor0 = new TalonFX(FlywheelConstants.kFlywheelMotorCanID0, Constants.kCANBus);
    motor1 = new TalonFX(FlywheelConstants.kFlywheelMotorCanID1, Constants.kCANBus);
    motor2 = new TalonFX(FlywheelConstants.kFlywheelMotorCanID2, Constants.kCANBus);
    motor3 = new TalonFX(FlywheelConstants.kFlywheelMotorCanID3, Constants.kCANBus);

    velocitySignal0 = motor0.getVelocity();
    voltsSignal0 = motor0.getMotorVoltage();
    currentTorqueSignal0 = motor0.getTorqueCurrent();

    velocitySignal1 = motor1.getVelocity();
    voltsSignal1 = motor1.getMotorVoltage();
    currentTorqueSignal1 = motor1.getTorqueCurrent();

    velocitySignal2 = motor2.getVelocity();
    voltsSignal2 = motor2.getMotorVoltage();
    currentTorqueSignal2 = motor2.getTorqueCurrent();

    velocitySignal3 = motor3.getVelocity();
    voltsSignal3 = motor3.getMotorVoltage();
    currentTorqueSignal3 = motor3.getTorqueCurrent();

    motor0.getConfigurator().apply(FlywheelConstants.getTalonFXConfig());
    motor1.getConfigurator().apply(FlywheelConstants.getTalonFXConfig());
    motor2.getConfigurator().apply(FlywheelConstants.getTalonFXConfig());
    motor3.getConfigurator().apply(FlywheelConstants.getTalonFXConfig());

    motor1.setControl(new Follower(motor0.getDeviceID(), MotorAlignmentValue.Aligned));
    motor2.setControl(new Follower(motor0.getDeviceID(), MotorAlignmentValue.Aligned));
    motor3.setControl(new Follower(motor0.getDeviceID(), MotorAlignmentValue.Aligned));

    BaseStatusSignal.setUpdateFrequencyForAll(
        50,
        velocitySignal0,
        voltsSignal0,
        currentTorqueSignal0,
        velocitySignal1,
        voltsSignal1,
        currentTorqueSignal1,
        velocitySignal2,
        voltsSignal2,
        currentTorqueSignal2,
        velocitySignal3,
        voltsSignal3,
        currentTorqueSignal3);

    motor0.optimizeBusUtilization();
    motor1.optimizeBusUtilization();
    motor2.optimizeBusUtilization();
    motor3.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(FlywheelInputs inputs) {
    BaseStatusSignal.refreshAll(
        velocitySignal0,
        voltsSignal0,
        currentTorqueSignal0,
        velocitySignal1,
        voltsSignal1,
        currentTorqueSignal1,
        velocitySignal2,
        voltsSignal2,
        currentTorqueSignal2,
        velocitySignal3,
        voltsSignal3,
        currentTorqueSignal3);

    inputs.velocity0 = velocitySignal0.getValue();
    inputs.appliedVolts0 = voltsSignal0.getValue();
    inputs.currentTorqueAmps0 = currentTorqueSignal0.getValue();

    inputs.velocity1 = velocitySignal1.getValue();
    inputs.appliedVolts1 = voltsSignal1.getValue();
    inputs.currentTorqueAmps1 = currentTorqueSignal1.getValue();

    inputs.velocity2 = velocitySignal2.getValue();
    inputs.appliedVolts2 = voltsSignal2.getValue();
    inputs.currentTorqueAmps2 = currentTorqueSignal2.getValue();

    inputs.velocity3 = velocitySignal3.getValue();
    inputs.appliedVolts3 = voltsSignal3.getValue();
    inputs.currentTorqueAmps3 = currentTorqueSignal3.getValue();
  }

  @Override
  public void setFlywheelVelocity(AngularVelocity velocity) {
    motor0.setControl(velocityControl.withVelocity(velocity));
  }

  @Override
  public void stop() {
    motor0.setControl(new CoastOut());
  }
}
