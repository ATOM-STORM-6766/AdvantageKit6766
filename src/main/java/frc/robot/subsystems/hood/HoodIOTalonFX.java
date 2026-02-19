package frc.robot.subsystems.hood;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

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

public class HoodIOTalonFX implements HoodIO {
  protected final TalonFX hoodMotor;
  private final PositionVoltage positionControl = new PositionVoltage(0);
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

    Angle hoodPosition = Radians.of(hoodPositionRadians);
    inputs.position = hoodPosition;
    double hoodVelocityRadPerSec =
        velocitySignal.getValue().in(RadiansPerSecond) / HoodConstants.kHoodGearRatio;
    inputs.velocity = Degrees.per(Second).of(Units.radiansToDegrees(hoodVelocityRadPerSec));
    inputs.appliedVolts = voltsSignal.getValue();
    inputs.currentStatorAmps = currentStatorSignal.getValue();
    inputs.currentSupplyAmps = currentSupplySignal.getValue();
  }

  @Override
  public void setPositionSetpoint(Angle setpoint) {
    double setpointRadians =
        MathUtil.clamp(
            setpoint.in(Radians),
            HoodConstants.kHoodMinPosition.in(Radians),
            HoodConstants.kHoodMaxPosition.in(Radians));
    double setpointRotations = Units.radiansToRotations(setpointRadians);
    double setpointRotor = setpointRotations * HoodConstants.kHoodGearRatio;

    hoodMotor.setControl(positionControl.withPosition(setpointRotor));
  }

  @Override
  public void setOpenloopVoltage(Voltage voltage) {
    hoodMotor.setControl(voltageControl.withOutput(voltage.in(Volts)));
  }

  @Override
  public void setRotorPosition(Angle hoodPosition) {
    double positionRotations = Units.radiansToRotations(hoodPosition.in(Radians));
    double positionRotor = positionRotations * HoodConstants.kHoodGearRatio;
    hoodMotor.setPosition(positionRotor);
  }
}
