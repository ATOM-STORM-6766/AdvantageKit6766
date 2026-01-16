package frc.robot.subsystems.hood;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotBase;
import org.littletonrobotics.junction.Logger;

public class HoodIOTalonFX implements HoodIO {
  protected final TalonFX hoodMotor;
  private final PositionVoltage positionControl = new PositionVoltage(0);
  private final StatusSignal<Angle> positionSignal;
  private final StatusSignal<AngularVelocity> velocitySignal;
  private final StatusSignal<Voltage> voltsSignal;
  private final StatusSignal<Current> currentStatorSignal;
  private final StatusSignal<Current> currentSupplySignal;

  public HoodIOTalonFX() {
    hoodMotor = new TalonFX(HoodConstants.kHoodMotorCanID, new CANBus("rio"));

    positionSignal = hoodMotor.getPosition();
    velocitySignal = hoodMotor.getVelocity();
    voltsSignal = hoodMotor.getMotorVoltage();
    currentStatorSignal = hoodMotor.getStatorCurrent();
    currentSupplySignal = hoodMotor.getSupplyCurrent();

    var config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
        Units.radiansToRotations(HoodConstants.kHoodMaxPositionRadians)
            / HoodConstants.kHoodGearRatio;
    config.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
        Units.radiansToRotations(HoodConstants.kHoodMinPositionRadians)
            / HoodConstants.kHoodGearRatio;

    config.Slot0.kS = HoodConstants.kS;
    config.Slot0.kP = HoodConstants.kP;
    config.Slot0.kD = HoodConstants.kD;
    config.Slot0.kV = HoodConstants.kV;
    config.Slot0.kA = HoodConstants.kA;

    if (RobotBase.isReal()) {
      config.CurrentLimits.StatorCurrentLimit = HoodConstants.kStatorCurrentLimit;
      config.CurrentLimits.StatorCurrentLimitEnable = HoodConstants.kStatorCurrentLimitEnable;
      config.ClosedLoopRamps = HoodConstants.makeClosedLoopRampConfig();
      config.OpenLoopRamps = HoodConstants.makeOpenLoopRampConfig();
    }

    config.MotionMagic.MotionMagicAcceleration = HoodConstants.kMotionMagicAcceleration;
    config.MotionMagic.MotionMagicCruiseVelocity = HoodConstants.kMotionMagicCruiseVelocity;

    hoodMotor.getConfigurator().apply(config);
    hoodMotor.setPosition(0.0);

    BaseStatusSignal.setUpdateFrequencyForAll(
        50, positionSignal, velocitySignal, voltsSignal, currentStatorSignal, currentSupplySignal);
    hoodMotor.optimizeBusUtilization();
  }

  @Override
  public void readInputs(HoodInputs inputs) {
    BaseStatusSignal.refreshAll(
        positionSignal, velocitySignal, voltsSignal, currentStatorSignal, currentSupplySignal);

    inputs.positionRad =
        Units.rotationsToRadians(positionSignal.getValueAsDouble() * HoodConstants.kHoodGearRatio);
    inputs.positionRotations = positionSignal.getValueAsDouble();
    inputs.velocityRadPerSec =
        Units.rotationsToRadians(velocitySignal.getValueAsDouble() * HoodConstants.kHoodGearRatio);
    inputs.appliedVolts = voltsSignal.getValueAsDouble();
    inputs.currentStatorAmps = currentStatorSignal.getValueAsDouble();
    inputs.currentSupplyAmps = currentSupplySignal.getValueAsDouble();
  }

  @Override
  public void setPositionSetpoint(double radiansFromCenter, double radsPerSec) {
    double setpointRadians =
        MathUtil.clamp(
            radiansFromCenter,
            HoodConstants.kHoodMinPositionRadians,
            HoodConstants.kHoodMaxPositionRadians);
    double setpointRotations = Units.radiansToRotations(setpointRadians);
    double setpointRotor = setpointRotations / HoodConstants.kHoodGearRatio;
    double ffVel = Units.radiansToRotations(radsPerSec) / HoodConstants.kHoodGearRatio;

    hoodMotor.setControl(positionControl.withPosition(setpointRotor).withVelocity(ffVel));

    Logger.recordOutput("Hood/IO/setPositionSetpoint/radiansFromCenter", radiansFromCenter);
    Logger.recordOutput("Hood/IO/setPositionSetpoint/radsPerSec", radsPerSec);
    Logger.recordOutput("Hood/IO/setPositionSetpoint/ffVel", ffVel);
    Logger.recordOutput("Hood/IO/setPositionSetpoint/setpointRotor", setpointRotor);
    Logger.recordOutput(
        "Hood/IO/setPositionSetpoint/radsPerSecRotor", radsPerSec / HoodConstants.kHoodGearRatio);
  }
}
