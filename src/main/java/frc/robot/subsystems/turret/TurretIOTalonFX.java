package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Rotations;

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

public class TurretIOTalonFX implements TurretIO {
  protected final TalonFX talon;
  private final PositionVoltage positionVoltageControl = new PositionVoltage(0.0).withSlot(0);
  private final StatusSignal<Angle> positionSignal;
  private final StatusSignal<AngularVelocity> velocitySignal;
  private final StatusSignal<Voltage> voltsSignal;
  private final StatusSignal<Current> currentStatorSignal;
  private final StatusSignal<Current> currentSupplySignal;

  public TurretIOTalonFX() {
    talon = new TalonFX(TurretConstants.kTurretMotorCanID, new CANBus("rio"));

    positionSignal = talon.getPosition();
    velocitySignal = talon.getVelocity();
    voltsSignal = talon.getMotorVoltage();
    currentStatorSignal = talon.getStatorCurrent();
    currentSupplySignal = talon.getSupplyCurrent();

    // Configure TalonFX
    var config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
        Units.radiansToRotations(TurretConstants.kTurretMaxPositionRadians)
            / TurretConstants.kTurretGearRatio;
    config.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
        Units.radiansToRotations(TurretConstants.kTurretMinPositionRadians)
            / TurretConstants.kTurretGearRatio;

    if (RobotBase.isReal()) {
      config.CurrentLimits.StatorCurrentLimit = TurretConstants.kStatorCurrentLimit;
      config.CurrentLimits.StatorCurrentLimitEnable = TurretConstants.kStatorCurrentLimitEnable;
      config.ClosedLoopRamps = TurretConstants.makeClosedLoopRampConfig();
      config.OpenLoopRamps = TurretConstants.makeOpenLoopRampConfig();
    }

    config.Slot0.kS = TurretConstants.kS;
    config.Slot0.kP = TurretConstants.kP;
    config.Slot0.kD = TurretConstants.kD;
    config.Slot0.kV = TurretConstants.kV;
    config.Slot0.kA = TurretConstants.kA;

    config.MotionMagic.MotionMagicJerk = TurretConstants.kMotionMagicJerk;
    config.MotionMagic.MotionMagicAcceleration = TurretConstants.kMotionMagicAcceleration;
    config.MotionMagic.MotionMagicCruiseVelocity = TurretConstants.kMotionMagicCruiseVelocity;

    talon.getConfigurator().apply(config);
    talon.setPosition(0.0);

    BaseStatusSignal.setUpdateFrequencyForAll(
        50, positionSignal, velocitySignal, voltsSignal, currentStatorSignal, currentSupplySignal);
    talon.optimizeBusUtilization();
  }

  @Override
  public void readInputs(TurretInputs inputs) {
    BaseStatusSignal.refreshAll(
        positionSignal, velocitySignal, voltsSignal, currentStatorSignal, currentSupplySignal);

    double talonPosition =
        BaseStatusSignal.getLatencyCompensatedValue(positionSignal, velocitySignal).in(Rotations);
    double kGearRatio = TurretConstants.kTurretGearRatio;

    inputs.positionRad = Units.rotationsToRadians(talonPosition * kGearRatio);
    inputs.velocityRadPerSec =
        Units.rotationsToRadians(velocitySignal.getValueAsDouble() * kGearRatio);
    inputs.appliedVolts = voltsSignal.getValueAsDouble();
    inputs.currentStatorAmps = currentStatorSignal.getValueAsDouble();
    inputs.currentSupplyAmps = currentSupplySignal.getValueAsDouble();
  }

  @Override
  public void setPositionSetpoint(double radiansFromCenter, double radsPerSecond) {
    double setpointRadians =
        MathUtil.clamp(
            radiansFromCenter,
            TurretConstants.kTurretMinPositionRadians,
            TurretConstants.kTurretMaxPositionRadians);
    double setpointRotations = Units.radiansToRotations(setpointRadians);
    double setpointRotor = setpointRotations / TurretConstants.kTurretGearRatio;
    double ffVel = Units.radiansToRotations(radsPerSecond) / TurretConstants.kTurretGearRatio;

    talon.setControl(positionVoltageControl.withPosition(setpointRotor).withVelocity(ffVel));

    Logger.recordOutput("Turret/IO/setPositionSetpoint/radiansFromCenter", radiansFromCenter);
    Logger.recordOutput("Turret/IO/setPositionSetpoint/radsPerSecond", radsPerSecond);
    Logger.recordOutput("Turret/IO/setPositionSetpoint/ffVel", ffVel);
    Logger.recordOutput("Turret/IO/setPositionSetpoint/setpointRotor", setpointRotor);
    Logger.recordOutput(
        "Turret/IO/setPositionSetpoint/radsPerSecondRotor",
        radsPerSecond / TurretConstants.kTurretGearRatio);
  }
}
