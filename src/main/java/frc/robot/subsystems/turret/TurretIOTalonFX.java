package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.generated.TunerConstants;
import java.util.Arrays;
import java.util.List;
import org.littletonrobotics.junction.Logger;

public class TurretIOTalonFX implements TurretIO {
  protected final TalonFX talon;
  protected final CANcoder canCoder;
  private final DutyCycleOut dutyCycleControl = new DutyCycleOut(0);
  private final PositionVoltage positionVoltageControl = new PositionVoltage(0.0).withSlot(0);
  private final StatusSignal<Angle> positionSignal;
  private final StatusSignal<AngularVelocity> velocitySignal;
  private final StatusSignal<Voltage> voltsSignal;
  private final StatusSignal<Current> currentStatorSignal;
  private final StatusSignal<Current> currentSupplySignal;
  private final StatusSignal<Angle> cancoderAbsolutePosition;
  private final StatusSignal<AngularVelocity> cancoderVelocity;
  private boolean cancoderOffset = false;

  public TurretIOTalonFX() {
    talon = new TalonFX(TurretConstants.kTurretMotorCanID, TunerConstants.kCANBus);
    canCoder = new CANcoder(TurretConstants.kTurretCanCoderID, TunerConstants.kCANBus);

    positionSignal = talon.getPosition();
    velocitySignal = talon.getVelocity();
    voltsSignal = talon.getMotorVoltage();
    currentStatorSignal = talon.getStatorCurrent();
    currentSupplySignal = talon.getSupplyCurrent();
    cancoderAbsolutePosition = canCoder.getPosition();
    cancoderVelocity = canCoder.getVelocity();

    // Configure CANcoder
    var cancoderConfig = new CANcoderConfiguration();
    cancoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
    cancoderConfig.MagnetSensor.MagnetOffset = -1.0 * TurretConstants.kTurretCancoderOffset;
    canCoder.getConfigurator().apply(cancoderConfig);

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

    BaseStatusSignal.setUpdateFrequencyForAll(
        50, voltsSignal, currentStatorSignal, currentSupplySignal);
    BaseStatusSignal.setUpdateFrequencyForAll(
        50, cancoderAbsolutePosition, cancoderVelocity, positionSignal, velocitySignal);
    talon.optimizeBusUtilization();
  }

  public List<BaseStatusSignal> getStatusSignals() {
    return Arrays.asList(
        positionSignal, velocitySignal, cancoderAbsolutePosition, cancoderVelocity);
  }

  @Override
  public void readInputs(TurretInputs inputs) {
    BaseStatusSignal.refreshAll(
        positionSignal,
        velocitySignal,
        voltsSignal,
        currentStatorSignal,
        currentSupplySignal,
        cancoderAbsolutePosition,
        cancoderVelocity);

    // Initialize talon position from CANcoder on first read
    if (!cancoderOffset && cancoderAbsolutePosition != null) {
      double cancoderPos = cancoderAbsolutePosition.getValueAsDouble();
      talon.setPosition(cancoderPos / TurretConstants.kTurretGearRatio);
      cancoderOffset = true;
    }
    Logger.recordOutput("Turret/IO/cancoderOffset", cancoderOffset);

    double talonPosition =
        BaseStatusSignal.getLatencyCompensatedValue(positionSignal, velocitySignal).in(Rotations);
    double kGearRatio = TurretConstants.kTurretGearRatio;

    inputs.positionRad = Units.rotationsToRadians(talonPosition * kGearRatio);
    inputs.velocityRadPerSec =
        Units.rotationsToRadians(velocitySignal.getValueAsDouble() * kGearRatio);
    inputs.turretPositionAbsolute =
        Rotation2d.fromRotations(
            BaseStatusSignal.getLatencyCompensatedValue(cancoderAbsolutePosition, cancoderVelocity)
                .in(Rotations));

    inputs.cancoderAbsolutePosition = cancoderAbsolutePosition.getValueAsDouble();
    inputs.appliedVolts = voltsSignal.getValueAsDouble();
    inputs.currentStatorAmps = currentStatorSignal.getValueAsDouble();
    inputs.currentSupplyAmps = currentSupplySignal.getValueAsDouble();
  }

  @Override
  public void setOpenLoopDutyCycle(double dutyCycle) {
    talon.setControl(dutyCycleControl.withOutput(dutyCycle));
    Logger.recordOutput("Turret/IO/setOpenLoopDutyCycle/dutyCycle", dutyCycle);
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
