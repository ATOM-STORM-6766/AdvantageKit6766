package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.util.PhoenixUtil;

public class IntakeIOTalonFX implements IntakeIO {
  private final TalonFX intakeMotor;
  private final TalonFX positionMotor;

  private final StatusSignal<Angle> positionSignal;
  private final StatusSignal<Current> positionCurrentSignal;
  private final StatusSignal<AngularVelocity> positionVelocitySignal;
  private final StatusSignal<AngularVelocity> velocitySignal;

  private final MotionMagicExpoVoltage positionControl =
      new MotionMagicExpoVoltage(0.0).withEnableFOC(true).withUseTimesync(true);

  private final VoltageOut velocityControl =
      new VoltageOut(3.648).withEnableFOC(true).withUseTimesync(true);

  public IntakeIOTalonFX() {
    intakeMotor = new TalonFX(IntakeConstants.intakeMotorID, IntakeConstants.canBus);
    positionMotor = new TalonFX(IntakeConstants.positionMotorID, IntakeConstants.canBus);

    // intakeMotor.getConfigurator().apply(IntakeConstants.getTalonFXConfig()); TODO
    PhoenixUtil.tryUntilOk(
        5, () -> positionMotor.getConfigurator().apply(IntakeConstants.getPositionConfig(), 0.25));

    positionSignal = positionMotor.getPosition();
    positionCurrentSignal = positionMotor.getTorqueCurrent();
    positionVelocitySignal = positionMotor.getVelocity();
    velocitySignal = intakeMotor.getVelocity();
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    positionSignal.refresh();
    positionCurrentSignal.refresh();
    positionVelocitySignal.refresh();
    velocitySignal.refresh();

    inputs.intakePosition = positionSignal.getValue();
    inputs.positionCurrent = positionCurrentSignal.getValue();
    inputs.positionVelocity = positionVelocitySignal.getValue();
    inputs.intakeVelocity = velocitySignal.getValue();
  }

  @Override
  public void setIntakePosition(Distance position) {
    positionMotor.setControl(
        positionControl.withPosition(
            Rotations.of(
                position.in(Meters) / IntakeConstants.positionMetersPerMechanismRotation)));
  }

  @Override
  public void setIntakeVelocity(Voltage voltage) {
    intakeMotor.setControl(velocityControl.withOutput(voltage));
  }

  @Override
  public void setPositionVoltage(Voltage voltage) {
    positionMotor.setControl(new VoltageOut(voltage));
  }

  @Override
  public void setPosition(double positionRotations) {
    positionMotor.setPosition(positionRotations);
  }

  @Override
  public void setPositionCurrent(Current current) {
    positionMotor.setControl(new TorqueCurrentFOC(current));
  }
}
