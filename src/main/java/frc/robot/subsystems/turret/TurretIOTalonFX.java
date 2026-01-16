package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.Rotations;

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
import frc.robot.Constants;

import org.littletonrobotics.junction.Logger;

public class TurretIOTalonFX implements TurretIO {
  protected final TalonFX talon;
  private final MotionMagicVoltage motionMagicVoltageControl = new MotionMagicVoltage(0.0);
  private final VoltageOut openLoopVoltageControl = new VoltageOut(0.0);
  private final StatusSignal<Angle> positionSignal;
  private final StatusSignal<AngularVelocity> velocitySignal;
  private final StatusSignal<Voltage> voltsSignal;
  private final StatusSignal<Current> currentStatorSignal;
  private final StatusSignal<Current> currentSupplySignal;

  public TurretIOTalonFX() {
    talon = new TalonFX(TurretConstants.kTurretMotorCanID, Constants.kCANBus);

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

    double talonPosition =
        BaseStatusSignal.getLatencyCompensatedValue(positionSignal, velocitySignal).in(Rotations);

    inputs.positionRad = Units.rotationsToRadians(talonPosition / TurretConstants.kTurretGearRatio);
    inputs.velocityRadPerSec =
        Units.rotationsToRadians(velocitySignal.getValueAsDouble() / TurretConstants.kTurretGearRatio);
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

    talon.setControl(motionMagicVoltageControl.withPosition(setpointRotor).withFeedForward(ffVel).withEnableFOC(true));

    Logger.recordOutput("Turret/IO/setPositionSetpoint/radiansFromCenter", radiansFromCenter);
    Logger.recordOutput("Turret/IO/setPositionSetpoint/radsPerSecond", radsPerSecond);
    Logger.recordOutput("Turret/IO/setPositionSetpoint/ffVel", ffVel);
    Logger.recordOutput("Turret/IO/setPositionSetpoint/setpointRotor", setpointRotor);
    Logger.recordOutput(
        "Turret/IO/setPositionSetpoint/radsPerSecondRotor",
        radsPerSecond * TurretConstants.kTurretGearRatio);
  }

  @Override
  public void setOpenLoopVoltage(double voltage) {
    talon.setControl(openLoopVoltageControl.withOutput(voltage));

    Logger.recordOutput("Turret/IO/setOpenLoopVoltage/voltage", voltage);
  }
}
