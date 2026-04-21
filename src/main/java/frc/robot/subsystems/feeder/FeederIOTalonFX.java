package frc.robot.subsystems.feeder;

import static edu.wpi.first.units.Units.RPM;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;

public class FeederIOTalonFX implements FeederIO {
  private final TalonFX shooterFeedMotor;
  private final TalonFX intakeFeedMotor;
  private final TalonFX intakeFeedFollowerMotor;

  private final StatusSignal<AngularVelocity> shooterVelocitySignal;
  private final StatusSignal<Voltage> shooterVoltsSignal;
  private final StatusSignal<Current> shooterCurrentSignal;

  private final StatusSignal<AngularVelocity> intakeVelocitySignal;
  private final StatusSignal<Voltage> intakeVoltsSignal;
  private final StatusSignal<Current> intakeCurrentSignal;

  private final VelocityTorqueCurrentFOC shooterVelocityControl =
      new VelocityTorqueCurrentFOC(0).withUseTimesync(true);

  private final DutyCycleOut intakeVelocityControl = new DutyCycleOut(0).withUseTimesync(true);

  public FeederIOTalonFX() {
    shooterFeedMotor = new TalonFX(FeederConstants.kShooterFeedMotorCanID, Constants.kCANBus);
    intakeFeedMotor = new TalonFX(FeederConstants.kIntakeFeedMotorCanID, Constants.kCANBus);
    intakeFeedFollowerMotor =
        new TalonFX(FeederConstants.kIntakeFeedFollowerMotorCanID, Constants.kCANBus);

    shooterVelocitySignal = shooterFeedMotor.getVelocity();
    shooterVoltsSignal = shooterFeedMotor.getMotorVoltage();
    shooterCurrentSignal = shooterFeedMotor.getTorqueCurrent();

    intakeVelocitySignal = intakeFeedMotor.getVelocity();
    intakeVoltsSignal = intakeFeedMotor.getMotorVoltage();
    intakeCurrentSignal = intakeFeedMotor.getTorqueCurrent();

    shooterFeedMotor.getConfigurator().apply(FeederConstants.getShooterFeedTalonFXConfig());
    intakeFeedMotor.getConfigurator().apply(FeederConstants.getIntakeFeedTalonFXConfig());
    intakeFeedFollowerMotor.getConfigurator().apply(FeederConstants.getIntakeFeedTalonFXConfig());

    intakeFeedFollowerMotor.setControl(
        new Follower(intakeFeedMotor.getDeviceID(), MotorAlignmentValue.Opposed));

    BaseStatusSignal.setUpdateFrequencyForAll(
        50,
        shooterVelocitySignal,
        shooterVoltsSignal,
        shooterCurrentSignal,
        intakeVelocitySignal,
        intakeVoltsSignal,
        intakeCurrentSignal);

    shooterFeedMotor.optimizeBusUtilization();
    intakeFeedMotor.optimizeBusUtilization();
    intakeFeedFollowerMotor.optimizeBusUtilization();
  }

  @Override
  public void readInputs(FeederInputs inputs) {
    if (BaseStatusSignal.waitForAll(
            0.0,
            shooterVelocitySignal,
            shooterVoltsSignal,
            shooterCurrentSignal,
            intakeVelocitySignal,
            intakeVoltsSignal,
            intakeCurrentSignal)
        .isOK()) {
      inputs.shooterVelocity = shooterVelocitySignal.getValue();
      inputs.shooterAppliedVolts = shooterVoltsSignal.getValue();
      inputs.shooterTorqueCurrent = shooterCurrentSignal.getValue();

      inputs.intakeVelocity = intakeVelocitySignal.getValue();
      inputs.intakeAppliedVolts = intakeVoltsSignal.getValue();
      inputs.intakeTorqueCurrent = intakeCurrentSignal.getValue();
    }
  }

  @Override
  public void setShooterVelocity(AngularVelocity velocity) {
    shooterFeedMotor.setControl(shooterVelocityControl.withVelocity(velocity));
  }

  @Override
  public void setIntakeVelocity(AngularVelocity velocity) {
    intakeFeedMotor.setControl(intakeVelocityControl.withOutput(velocity.in(RPM) / 6065));
  }

  @Override
  public void stop() {
    shooterFeedMotor.setControl(new CoastOut());
    intakeFeedMotor.setControl(new CoastOut());
  }
}
