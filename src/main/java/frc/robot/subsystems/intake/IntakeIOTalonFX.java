package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;

public class IntakeIOTalonFX implements IntakeIO {
  protected final TalonFX rollerMotor;
  protected final TalonFX rollerFollowerMotor;
  protected final TalonFX positionMotor;

  private final StatusSignal<Angle> positionSignal;
  private final StatusSignal<Current> positionStatorCurrentSignal;
  private final StatusSignal<Current> positionSupplyCurrentSignal;
  private final StatusSignal<AngularVelocity> positionVelocitySignal;
  private final StatusSignal<AngularVelocity> velocitySignal;
  private final StatusSignal<AngularVelocity> velocityFollowerSignal;
  private final StatusSignal<Voltage> voltageSignal;
  private final StatusSignal<Voltage> voltageFollowerSignal;

  private final MotionMagicTorqueCurrentFOC positionControl =
      new MotionMagicTorqueCurrentFOC(0.0).withUseTimesync(true);

  private final VelocityTorqueCurrentFOC positionWithVelocityControl =
      new VelocityTorqueCurrentFOC(0.0).withUseTimesync(true).withSlot(1);

  private final VoltageOut velocityControl =
      new VoltageOut(0).withEnableFOC(true).withUseTimesync(true);

  public IntakeIOTalonFX() {
    rollerMotor = new TalonFX(IntakeConstants.intakeMotorID, Constants.kCANBus);
    rollerFollowerMotor = new TalonFX(IntakeConstants.rollerFollowerMotorID, Constants.kCANBus);
    positionMotor = new TalonFX(IntakeConstants.positionMotorID, Constants.kCANBus);

    positionSignal = positionMotor.getPosition();

    positionStatorCurrentSignal = positionMotor.getStatorCurrent();
    positionSupplyCurrentSignal = positionMotor.getSupplyCurrent();

    positionVelocitySignal = positionMotor.getVelocity();
    velocitySignal = rollerMotor.getVelocity();
    velocityFollowerSignal = rollerFollowerMotor.getVelocity();
    voltageSignal = rollerMotor.getMotorVoltage();
    voltageFollowerSignal = rollerFollowerMotor.getMotorVoltage();

    rollerMotor.getConfigurator().apply(IntakeConstants.getRollerConfig());
    rollerFollowerMotor.getConfigurator().apply(IntakeConstants.getRollerConfig());
    rollerFollowerMotor.setControl(
        new Follower(rollerMotor.getDeviceID(), MotorAlignmentValue.Opposed));
    positionMotor.getConfigurator().apply(IntakeConstants.getPositionConfig());
    positionMotor.setPosition(0.0);

    BaseStatusSignal.setUpdateFrequencyForAll(
        50,
        positionSignal,
        positionStatorCurrentSignal,
        positionSupplyCurrentSignal,
        positionVelocitySignal,
        velocitySignal,
        velocityFollowerSignal,
        voltageSignal,
        voltageFollowerSignal);
    positionMotor.optimizeBusUtilization();
    rollerMotor.optimizeBusUtilization();
    rollerFollowerMotor.optimizeBusUtilization();
  }

  @Override
  public void readInputs(IntakeIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        positionSignal,
        positionStatorCurrentSignal,
        positionSupplyCurrentSignal,
        positionVelocitySignal,
        velocitySignal,
        velocityFollowerSignal,
        voltageSignal,
        voltageFollowerSignal);

    inputs.intakePositionRotation = positionSignal.getValue();
    inputs.intakePosition = IntakeConstants.rotationToDistance(inputs.intakePositionRotation);
    inputs.positionStatorAmps = positionStatorCurrentSignal.getValue();
    inputs.positionSupplyAmps = positionSupplyCurrentSignal.getValue();
    inputs.positionVelocity = positionVelocitySignal.getValue();
    inputs.intakeVelocity0 = velocitySignal.getValue();
    inputs.intakeVelocity1 = velocityFollowerSignal.getValue();
  }

  @Override
  public void setIntakePosition(Angle position) {
    double setpointMechanismRotations =
        MathUtil.clamp(
            position.in(Rotations),
            IntakeConstants.kIntakeMinMechanismRotations,
            IntakeConstants.kIntakeMaxMechanismRotations);
    positionMotor.setControl(positionControl.withPosition(setpointMechanismRotations));
  }

  @Override
  public void setIntakePositionWithVelocity(Angle position, double velocityRPS) {
    positionMotor.setControl(positionWithVelocityControl.withVelocity(velocityRPS));
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
  public void setIntakeSensorPosition(Angle position) {
    positionMotor.setPosition(position);
  }
}
