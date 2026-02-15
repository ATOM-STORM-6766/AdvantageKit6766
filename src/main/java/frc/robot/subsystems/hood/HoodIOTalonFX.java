package frc.robot.subsystems.hood;

import static edu.wpi.first.units.Units.Radians;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;

public class HoodIOTalonFX implements HoodIO {
  protected final TalonFX hoodMotor;
  private final PositionTorqueCurrentFOC positionControl = new PositionTorqueCurrentFOC(0);
  private final VoltageOut voltageControl = new VoltageOut(0);
  private final StatusSignal<Angle> positionSignal;
  private final StatusSignal<AngularVelocity> velocitySignal;
  private final StatusSignal<Voltage> voltsSignal;
  private final StatusSignal<Current> currentStatorSignal;
  private final StatusSignal<Current> currentSupplySignal;

  public HoodIOTalonFX() {
    hoodMotor = new TalonFX(HoodConstants.kHoodMotorCanID, Constants.kCANBus);

    positionSignal = hoodMotor.getPosition();
    velocitySignal = hoodMotor.getVelocity();
    voltsSignal = hoodMotor.getMotorVoltage();
    currentStatorSignal = hoodMotor.getStatorCurrent();
    currentSupplySignal = hoodMotor.getSupplyCurrent();

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

    double hoodRotorPositionRadians =
        BaseStatusSignal.getLatencyCompensatedValue(positionSignal, velocitySignal).in(Radians);
    double hoodPositionRadians = (hoodRotorPositionRadians / HoodConstants.kHoodGearRatio);

    inputs.positionRad = hoodPositionRadians;
    inputs.positionDegrees = Units.radiansToDegrees(hoodPositionRadians);
    inputs.velocityRadPerSec =
        Units.radiansToRotations(velocitySignal.getValueAsDouble() / HoodConstants.kHoodGearRatio);
    inputs.velocityDegreesPerSec =
        Units.radiansToDegrees(velocitySignal.getValueAsDouble() / HoodConstants.kHoodGearRatio);
    inputs.appliedVolts = voltsSignal.getValueAsDouble();
    inputs.currentStatorAmps = currentStatorSignal.getValueAsDouble();
    inputs.currentSupplyAmps = currentSupplySignal.getValueAsDouble();
  }

  @Override
  public void setPositionSetpoint(double setpoint) {
    double setpointRadians =
        MathUtil.clamp(
            setpoint, HoodConstants.kHoodMinPositionRadians, HoodConstants.kHoodMaxPositionRadians);
    double setpointRotations = Units.radiansToRotations(setpointRadians);
    double setpointRotor = setpointRotations * HoodConstants.kHoodGearRatio;

    hoodMotor.setControl(positionControl.withPosition(setpointRotor));
  }

  @Override
  public void setOpenloopVoltage(double voltage) {
    hoodMotor.setControl(voltageControl.withOutput(voltage));
  }

  @Override
  public void setRotorPosition(double hoodPositionRadians) {
    double positionRotations = Units.radiansToRotations(hoodPositionRadians);
    double positionRotor = positionRotations * HoodConstants.kHoodGearRatio;
    hoodMotor.setPosition(positionRotor);
  }
}
