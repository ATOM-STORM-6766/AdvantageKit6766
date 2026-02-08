package frc.robot.subsystems.intake;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;

public class IntakeIOTalonFX implements IntakeIO {

  private final TalonFX intakeMotor;
  private final TalonFX positionMotor;
  private final TalonFX feedMotor;

  private final StatusSignal<Angle> positionSignal;
  private final StatusSignal<Current> positionCurrentSignal;
  private final StatusSignal<AngularVelocity> positionVelocitySignal;
  private final StatusSignal<AngularVelocity> velocitySignal;
  private final StatusSignal<AngularVelocity> feedVelocitySignal;

  private final MotionMagicExpoVoltage positionControl =
      new MotionMagicExpoVoltage(0.0).withEnableFOC(true).withUseTimesync(true);

  private final VoltageOut velocityControl =
      new VoltageOut(3.648).withEnableFOC(true).withUseTimesync(true);

  private final VelocityTorqueCurrentFOC feedVelocityControl =
      new VelocityTorqueCurrentFOC(0).withUseTimesync(true).withSlot(1);

  public IntakeIOTalonFX() {
    intakeMotor = new TalonFX(IntakeConstants.intakeMotorID, IntakeConstants.canBus);
    positionMotor = new TalonFX(IntakeConstants.positionMotorID, IntakeConstants.canBus);
    feedMotor = new TalonFX(IntakeConstants.feedMotorID, IntakeConstants.canBus);

    intakeMotor.getConfigurator().apply(IntakeConstants.getTalonFXConfig());
    positionMotor.getConfigurator().apply(IntakeConstants.getPositionConfig());
    feedMotor.getConfigurator().apply(IntakeConstants.getTalonFXConfig());

    positionSignal = positionMotor.getPosition();
    positionCurrentSignal = positionMotor.getTorqueCurrent();
    positionVelocitySignal = positionMotor.getVelocity();
    velocitySignal = intakeMotor.getVelocity();
    feedVelocitySignal = feedMotor.getVelocity();
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    positionSignal.refresh();
    positionCurrentSignal.refresh();
    positionVelocitySignal.refresh();
    velocitySignal.refresh();
    feedVelocitySignal.refresh();

    inputs.intakePosition = new Rotation2d(positionSignal.getValue());
    inputs.positionCurrent = positionCurrentSignal.getValueAsDouble();
    inputs.positionVelocity = positionVelocitySignal.getValue();
    inputs.intakeVelocity = velocitySignal.getValueAsDouble();
    inputs.feedVelocity = feedVelocitySignal.getValueAsDouble();
  }

  @Override
  public void setIntakePosition(Angle position) {
    positionMotor.setControl(positionControl.withPosition(position));
  }

  @Override
  public void setIntakeVelocity(double voltage) {
    intakeMotor.setControl(velocityControl.withOutput(voltage));
  }

  @Override
  public void setFeedVelocity(double velocityRadPerSec) {
    feedMotor.set(
        velocityRadPerSec); // setControl(feedVelocityControl.withVelocity(velocityRadPerSec));
    // //TODO
  }

  @Override
  public void setPositionVoltage(double voltage) {
    positionMotor.setControl(new VoltageOut(voltage));
  }

  @Override
  public void setCurrentPosition(double positionRotations) {
    positionMotor.setPosition(positionRotations);
  }
}
