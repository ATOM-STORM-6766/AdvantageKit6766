package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
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

  private final MotionMagicTorqueCurrentFOC positionControl =
      new MotionMagicTorqueCurrentFOC(0.0).withUseTimesync(true);

  private final VoltageOut velocityControl =
      new VoltageOut(3.648).withEnableFOC(true).withUseTimesync(true);

  public IntakeIOTalonFX() {
    rollerMotor = new TalonFX(IntakeConstants.intakeMotorID, Constants.kCANBus);
    rollerFollowerMotor = new TalonFX(IntakeConstants.rollerFollowerMotorID, Constants.kCANBus);
    positionMotor = new TalonFX(IntakeConstants.positionMotorID, Constants.kCANBus);

    positionSignal = positionMotor.getPosition();

    positionStatorCurrentSignal = positionMotor.getStatorCurrent();
    positionSupplyCurrentSignal = positionMotor.getSupplyCurrent();

    positionVelocitySignal = positionMotor.getVelocity();
    velocitySignal = rollerMotor.getVelocity();

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
        velocitySignal);
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
        velocitySignal);

    double positionMechanismRotations =
        BaseStatusSignal.getLatencyCompensatedValue(positionSignal, positionVelocitySignal)
            .in(Rotations);
    double positionMeters =
        positionMechanismRotations * IntakeConstants.kPositionMetersPerMechanismRotation;
    double positionMechanismRadPerSec = positionVelocitySignal.getValue().in(RadiansPerSecond);

    inputs.intakePosition = Meters.of(positionMeters);
    inputs.positionStatorAmps = positionStatorCurrentSignal.getValue();
    inputs.positionSupplyAmps = positionSupplyCurrentSignal.getValue();
    inputs.positionVelocity = RadiansPerSecond.of(positionMechanismRadPerSec);
    inputs.intakeVelocity = velocitySignal.getValue();
  }

  @Override
  public void setIntakePosition(Distance position) {
    double setpointMechanismRotations =
        MathUtil.clamp(
            position.in(Meters) / IntakeConstants.kPositionMetersPerMechanismRotation,
            IntakeConstants.kIntakeMinMechanismRotations,
            IntakeConstants.kIntakeMaxMechanismRotations);
    positionMotor.setControl(positionControl.withPosition(setpointMechanismRotations));
  }

  @Override
  public void setIntakeVelocity(Voltage voltage) {
    rollerMotor.setControl(velocityControl.withOutput(voltage));
  }

  @Override
  public void setPositionVoltage(Voltage voltage) {
    positionMotor.setControl(new VoltageOut(voltage));
  }
}
