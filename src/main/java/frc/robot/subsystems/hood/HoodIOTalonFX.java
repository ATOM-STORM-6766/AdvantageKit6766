package frc.robot.subsystems.hood;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

public class HoodIOTalonFX implements HoodIO {
  protected final TalonFX hoodMotor;
  private final PositionVoltage positionControl = new PositionVoltage(0);
  private final VoltageOut voltageControl = new VoltageOut(0);
  private final StatusSignal<Angle> positionSignal;
  private final StatusSignal<AngularVelocity> velocitySignal;
  private final StatusSignal<Voltage> voltsSignal;
  private final StatusSignal<Current> currentStatorSignal;
  private final StatusSignal<Current> currentSupplySignal;
  protected CalibrationState calibrationState = CalibrationState.NOT_CALIBRATED;

  public HoodIOTalonFX() {
    hoodMotor = new TalonFX(HoodConstants.kHoodMotorCanID, Constants.kCANBus);

    positionSignal = hoodMotor.getPosition();
    velocitySignal = hoodMotor.getVelocity();
    voltsSignal = hoodMotor.getMotorVoltage();
    currentStatorSignal = hoodMotor.getStatorCurrent();
    currentSupplySignal = hoodMotor.getSupplyCurrent();

    // Configure TalonFX
    hoodMotor.getConfigurator().apply(HoodConstants.getTalonFXConfig());
    hoodMotor.setPosition(0.0);

    BaseStatusSignal.setUpdateFrequencyForAll(
        50, positionSignal, velocitySignal, voltsSignal, currentStatorSignal, currentSupplySignal);
    hoodMotor.optimizeBusUtilization();
  }

  @Override
  public void readInputs(HoodInputs inputs) {
    BaseStatusSignal.refreshAll(
        positionSignal, velocitySignal, voltsSignal, currentStatorSignal, currentSupplySignal);

    if (calibrationState == CalibrationState.CALIBRATING) {
      double currentAmps = currentStatorSignal.getValueAsDouble();
      if (currentAmps >= HoodConstants.kCalibrationCurrentThreshold) {
        hoodMotor.setControl(voltageControl.withOutput(0.0));

        double positionRotations = Units.radiansToRotations(HoodConstants.kHoodMinPositionRadians);
        double positionRotor = positionRotations * HoodConstants.kHoodGearRatio;
        hoodMotor.setPosition(positionRotor);

        calibrationState = CalibrationState.CALIBRATED;
      }

      inputs.appliedVolts = voltsSignal.getValueAsDouble();
      inputs.currentStatorAmps = currentStatorSignal.getValueAsDouble();
      inputs.currentSupplyAmps = currentSupplySignal.getValueAsDouble();
    } else {
      inputs.positionRad =
          Units.rotationsToRadians(
              positionSignal.getValueAsDouble() / HoodConstants.kHoodGearRatio);
      inputs.positionRotations = positionSignal.getValueAsDouble();
      inputs.velocityRadPerSec =
          Units.rotationsToRadians(
              velocitySignal.getValueAsDouble() / HoodConstants.kHoodGearRatio);
      inputs.appliedVolts = voltsSignal.getValueAsDouble();
      inputs.currentStatorAmps = currentStatorSignal.getValueAsDouble();
      inputs.currentSupplyAmps = currentSupplySignal.getValueAsDouble();
    }

    inputs.calibrationState = calibrationState;

    Logger.recordOutput("Hood/IO/Calibration", calibrationState);
  }

  @Override
  public void setPositionSetpoint(double radiansFromCenter, double radsPerSec) {
    double setpointRadians =
        MathUtil.clamp(
            radiansFromCenter,
            HoodConstants.kHoodMinPositionRadians,
            HoodConstants.kHoodMaxPositionRadians);
    double setpointRotations = Units.radiansToRotations(setpointRadians);
    double setpointRotor = setpointRotations * HoodConstants.kHoodGearRatio;
    double ffVel = Units.radiansToRotations(radsPerSec) * HoodConstants.kHoodGearRatio;

    hoodMotor.setControl(positionControl.withPosition(setpointRotor).withVelocity(ffVel));

    Logger.recordOutput("Hood/IO/setPositionSetpoint/radiansFromCenter", radiansFromCenter);
    Logger.recordOutput("Hood/IO/setPositionSetpoint/radsPerSec", radsPerSec);
    Logger.recordOutput("Hood/IO/setPositionSetpoint/ffVel", ffVel);
    Logger.recordOutput("Hood/IO/setPositionSetpoint/setpointRotor", setpointRotor);
    Logger.recordOutput(
        "Hood/IO/setPositionSetpoint/radsPerSecRotor", radsPerSec * HoodConstants.kHoodGearRatio);
  }

  @Override
  public void startCalibration() {
    calibrationState = CalibrationState.CALIBRATING;
    hoodMotor.setControl(voltageControl.withOutput(HoodConstants.kCalibrationVoltage));
  }
}
