package frc.robot.subsystems.intake;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

public class IntakeIOTalonFX implements IntakeIO {

  private final TalonFX motor;
  private final StatusSignal<Angle> positionSignal;
  private final StatusSignal<Current> currentSignal;
  private final StatusSignal<Voltage> voltageSignal;

  private final MotionMagicExpoVoltage positionControl =
      new MotionMagicExpoVoltage(0.0).withEnableFOC(true).withUseTimesync(true);

  public IntakeIOTalonFX() {
    motor = new TalonFX(IntakeConstants.motorID, IntakeConstants.canBus);
    positionSignal = motor.getPosition();
    currentSignal = motor.getSupplyCurrent();
    voltageSignal = motor.getSupplyVoltage();

    
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    positionSignal.refresh();
    currentSignal.refresh();
    voltageSignal.refresh();

    inputs.position = new Rotation2d(positionSignal.getValue());
    inputs.currentAmps = currentSignal.getValueAsDouble();
    inputs.voltageVolts = voltageSignal.getValueAsDouble();
  }

  @Override
  public void setPosition(Rotation2d position) {
    motor.setControl(positionControl);
}
}
