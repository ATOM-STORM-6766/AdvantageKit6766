package frc.robot.subsystems.hood;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.generated.TunerConstants;
import org.littletonrobotics.junction.Logger;

public class HoodIOTalonFX implements HoodIO {
  protected final TalonFX hoodMotor;
  private final PositionVoltage positionControl = new PositionVoltage(0);
  private final DutyCycleOut dutyCycleOut = new DutyCycleOut(0);
  private final StatusSignal<Angle> positionSignal;
  private final StatusSignal<AngularVelocity> velocitySignal;
  private final StatusSignal<Voltage> voltsSignal;
  private final StatusSignal<Current> currentStatorSignal;
  private final StatusSignal<Current> currentSupplySignal;

  public HoodIOTalonFX() {
    hoodMotor = new TalonFX(HoodConstants.kHoodMotorCanID, TunerConstants.kCANBus);

    positionSignal = hoodMotor.getPosition();
    velocitySignal = hoodMotor.getVelocity();
    voltsSignal = hoodMotor.getMotorVoltage();
    currentStatorSignal = hoodMotor.getStatorCurrent();
    currentSupplySignal = hoodMotor.getSupplyCurrent();

    var config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    config.Audio.BeepOnBoot = false;
    config.Audio.BeepOnConfig = false;

    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
        HoodConstants.kHoodRotorMaxPosition - HoodConstants.kHoodPositionTolerance;
    config.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
        HoodConstants.kHoodRotorMinPosition + HoodConstants.kHoodPositionTolerance;

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
  public void setNeutralMode(NeutralModeValue mode) {
    hoodMotor.setNeutralMode(mode);
  }

  @Override
  public void setPositionSetpoint(double radiansFromCenter, double radsPerSec) {
    double setpointRotations =
        Units.radiansToRotations(radiansFromCenter) / HoodConstants.kHoodGearRatio;
    double setpointRotor =
        MathUtil.clamp(
            setpointRotations,
            HoodConstants.kHoodRotorMinPosition + HoodConstants.kHoodPositionTolerance,
            HoodConstants.kHoodRotorMaxPosition - HoodConstants.kHoodPositionTolerance);
    double rotationsPerSec = Units.radiansToRotations(radsPerSec) / HoodConstants.kHoodGearRatio;

    hoodMotor.setControl(positionControl.withPosition(setpointRotor).withVelocity(rotationsPerSec));

    Logger.recordOutput("Hood/IO/setPositionSetpoint/radiansFromCenter", radiansFromCenter);
    Logger.recordOutput("Hood/IO/setPositionSetpoint/setpointRotor", setpointRotor);
  }

  @Override
  public void setDutyCycleOut(double percentOutput) {
    hoodMotor.setControl(dutyCycleOut.withOutput(percentOutput));
  }

  @Override
  public void resetZeroPoint() {
    hoodMotor.setPosition(0.0);
  }
}
