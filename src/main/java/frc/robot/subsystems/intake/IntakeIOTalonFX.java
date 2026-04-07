package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;

public class IntakeIOTalonFX implements IntakeIO {
  protected final TalonFX rollerMotor;
  protected final TalonFX positionMotor;

  private final StatusSignal<Angle> positionSignal;
  private final StatusSignal<Current> positionCurrentSignal;
  private final StatusSignal<AngularVelocity> positionVelocitySignal;
  private final StatusSignal<AngularVelocity> velocitySignal;

  private final MotionMagicExpoVoltage positionControl =
      new MotionMagicExpoVoltage(0.0).withEnableFOC(true).withUseTimesync(true);

  private final VoltageOut velocityControl =
      new VoltageOut(3.648).withEnableFOC(true).withUseTimesync(true);

  public IntakeIOTalonFX() {
    rollerMotor = new TalonFX(IntakeConstants.intakeMotorID, Constants.kCANBus);
    positionMotor = new TalonFX(IntakeConstants.positionMotorID, Constants.kCANBus);

    positionSignal = positionMotor.getPosition();
    positionCurrentSignal = positionMotor.getTorqueCurrent();
    positionVelocitySignal = positionMotor.getVelocity();
    velocitySignal = rollerMotor.getVelocity();

    rollerMotor.getConfigurator().apply(IntakeConstants.getRollerConfig());
    positionMotor.getConfigurator().apply(IntakeConstants.getPositionConfig());
    positionMotor.setPosition(0.0);

    BaseStatusSignal.setUpdateFrequencyForAll(
        50, positionSignal, positionCurrentSignal, positionVelocitySignal, velocitySignal);
    positionMotor.optimizeBusUtilization();
    rollerMotor.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        positionSignal, positionCurrentSignal, positionVelocitySignal, velocitySignal);

    double positionRotorRotations =
        BaseStatusSignal.getLatencyCompensatedValue(positionSignal, positionVelocitySignal)
            .in(Rotations);
    double positionMechanismRotations = positionRotorRotations / IntakeConstants.kPositionGearRatio;
    double positionMeters =
        positionMechanismRotations * IntakeConstants.kPositionMetersPerMechanismRotation;
    double positionMechanismRadPerSec =
        positionVelocitySignal.getValue().in(RadiansPerSecond) / IntakeConstants.kPositionGearRatio;

    inputs.intakePosition = Meters.of(positionMeters);
    inputs.positionCurrent = positionCurrentSignal.getValue();
    inputs.positionVelocity = RadiansPerSecond.of(positionMechanismRadPerSec);
    inputs.intakeVelocity = velocitySignal.getValue().div(IntakeConstants.kRollerGearRatio);
  }

  @Override
  public void setIntakePosition(Distance position) {
    double setpointMechanismRotations =
        MathUtil.clamp(
            position.in(Meters) / IntakeConstants.kPositionMetersPerMechanismRotation,
            IntakeConstants.kIntakeMinMechanismRotations,
            IntakeConstants.kIntakeMaxMechanismRotations);
    double setpointRotor = setpointMechanismRotations * IntakeConstants.kPositionGearRatio;

    positionMotor.setControl(positionControl.withPosition(setpointRotor));
  }

  @Override
  public void setIntakeVelocity(Voltage voltage) {
    rollerMotor.setControl(velocityControl.withOutput(voltage));
  }

  @Override
  public void setPositionVoltage(Voltage voltage) {
    positionMotor.setControl(new VoltageOut(voltage));
  }

  @Override
  public void setPositionCurrent(Current current) {
    positionMotor.setControl(new TorqueCurrentFOC(current));
  }
}
