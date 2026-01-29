package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.Radians;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

public class TurretIOTalonFX implements TurretIO {
  protected final TalonFX talon;
  protected final DutyCycleEncoder absoluteEncoder;
  protected final DutyCycleEncoder absoluteEncoder2;

  private final MotionMagicVoltage motionMagicVoltageControl = new MotionMagicVoltage(0.0);
  private final VoltageOut openLoopVoltageControl = new VoltageOut(0.0);
  private final StatusSignal<Angle> positionSignal;
  private final StatusSignal<AngularVelocity> velocitySignal;
  private final StatusSignal<Voltage> voltsSignal;
  private final StatusSignal<Current> currentStatorSignal;
  private final StatusSignal<Current> currentSupplySignal;

  public TurretIOTalonFX() {
    talon = new TalonFX(TurretConstants.kTurretMotorCanID, Constants.kCANBus);
    absoluteEncoder = new DutyCycleEncoder(TurretConstants.kAbsoluteEncoderDIO, 1, 0.27417);
    absoluteEncoder2 = new DutyCycleEncoder(TurretConstants.kAbsoluteEncoder2DIO, 1, 0.4908060000000001);

    positionSignal = talon.getPosition();
    velocitySignal = talon.getVelocity();
    voltsSignal = talon.getMotorVoltage();
    currentStatorSignal = talon.getStatorCurrent();
    currentSupplySignal = talon.getSupplyCurrent();

    // Configure TalonFX
    talon.getConfigurator().apply(TurretConstants.getTalonFXConfig());
    talon.setPosition(0.0);

    BaseStatusSignal.setUpdateFrequencyForAll(
        50, positionSignal, velocitySignal, voltsSignal, currentStatorSignal, currentSupplySignal);
    talon.optimizeBusUtilization();
  }

  @Override
  public void readInputs(TurretInputs inputs) {
    BaseStatusSignal.refreshAll(
        positionSignal, velocitySignal, voltsSignal, currentStatorSignal, currentSupplySignal);

    double turretRotorPositionRadians =
        BaseStatusSignal.getLatencyCompensatedValue(positionSignal, velocitySignal).in(Radians);
    double turretPositionRadians = (turretRotorPositionRadians / TurretConstants.kTurretGearRatio);

    inputs.positionRad = turretPositionRadians;
    inputs.positionDegrees = Units.radiansToDegrees(turretPositionRadians);
    inputs.velocityRadPerSec =
        Units.radiansToRotations(
            velocitySignal.getValueAsDouble() / TurretConstants.kTurretGearRatio);
    inputs.velocityDegreesPerSec =
        Units.radiansToDegrees(
            velocitySignal.getValueAsDouble() / TurretConstants.kTurretGearRatio);

    inputs.rotorPosition = Units.radiansToRotations(turretRotorPositionRadians);
    inputs.rotorPositionRadians = turretRotorPositionRadians;

    double absoluteEncoderPosition = absoluteEncoder.get();
    inputs.absoluteEncoderPosition = absoluteEncoderPosition;
    inputs.absoluteEncoderPositionRadians =
        Units.rotationsToRadians(
            absoluteEncoderPosition / TurretConstants.kTurretAbsoluteEncoderToTurretRatio);

    double absoluteEncoder2Position = absoluteEncoder2.get();
    inputs.absoluteEncoder2Position = absoluteEncoder2Position;
    inputs.absoluteEncoder2PositionRadians =
        Units.rotationsToRadians(
            absoluteEncoder2Position / TurretConstants.kTurretAbsoluteEncoder2ToTurretRatio);

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
    double setpointRotor = setpointRotations * TurretConstants.kTurretGearRatio;
    double ffVel = Units.radiansToRotations(radsPerSecond) * TurretConstants.kTurretGearRatio;

    talon.setControl(
        motionMagicVoltageControl
            .withPosition(setpointRotor)
            .withFeedForward(ffVel)
            .withEnableFOC(true));

    Logger.recordOutput("Turret/IO/setPositionSetpoint/radiansFromCenter", radiansFromCenter);
    Logger.recordOutput("Turret/IO/setPositionSetpoint/radsPerSecond", radsPerSecond);
    Logger.recordOutput("Turret/IO/setPositionSetpoint/ffVel", ffVel);
    Logger.recordOutput("Turret/IO/setPositionSetpoint/setpointRotor", setpointRotor);
    Logger.recordOutput(
        "Turret/IO/setPositionSetpoint/radsPerSecondRotor",
        radsPerSecond * TurretConstants.kTurretGearRatio);
  }

  @Override
  public void setOpenloopVoltage(double voltage) {
    talon.setControl(openLoopVoltageControl.withOutput(voltage));
  }

  @Override
  public void setRotorPosition(double turretPositionRadians) {
    double positionRotations = Units.radiansToRotations(turretPositionRadians);
    double positionRotor = positionRotations * TurretConstants.kTurretGearRatio;
    talon.setPosition(positionRotor);
  }
}
