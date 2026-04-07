package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Amp;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class IntakeIOSim extends IntakeIOTalonFX {
  private static final DCMotor POSITION_MOTOR = DCMotor.getKrakenX60(1);
  private static final DCMotor INTAKE_MOTOR = DCMotor.getKrakenX44(1);
  private static final double POSITION_MOMENT_OF_INERTIA = 0.05;
  private static final double INTAKE_MOMENT_OF_INERTIA = 0.005;
  private static final double SIM_STALL_CURRENT_AMPS =
      IntakeConstants.kCalibrationCurrentThreshold + 5.0;

  private final DCMotorSim positionSim;
  private final DCMotorSim intakeSim;
  private final Notifier simNotifier;
  private double lastUpdateTimestamp;
  private boolean simulatedStall = false;

  public IntakeIOSim() {
    super();

    positionSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                POSITION_MOTOR, POSITION_MOMENT_OF_INERTIA, IntakeConstants.kPositionGearRatio),
            POSITION_MOTOR);
    intakeSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(INTAKE_MOTOR, INTAKE_MOMENT_OF_INERTIA, 1.0),
            INTAKE_MOTOR);

    positionSim.setState(
        Units.rotationsToRadians(IntakeConstants.kIntakeMinMechanismRotations), 0.0);

    lastUpdateTimestamp = Timer.getFPGATimestamp();
    simNotifier = new Notifier(this::updateSimState);
    simNotifier.startPeriodic(0.005);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    super.updateInputs(inputs);

    if (simulatedStall) {
      inputs.positionCurrent = Amp.of(SIM_STALL_CURRENT_AMPS);
      inputs.positionVelocity = RadiansPerSecond.of(0.0);
    }
  }

  private void updateSimState() {
    var positionSimState = positionMotor.getSimState();
    var intakeSimState = rollerMotor.getSimState();

    positionSimState.setSupplyVoltage(12.0);
    intakeSimState.setSupplyVoltage(12.0);

    double positionVoltage = MathUtil.clamp(positionSimState.getMotorVoltage(), -12.0, 12.0);
    double intakeVoltage = MathUtil.clamp(intakeSimState.getMotorVoltage(), -12.0, 12.0);

    positionSim.setInputVoltage(positionVoltage);
    intakeSim.setInputVoltage(intakeVoltage);

    double timestamp = Timer.getFPGATimestamp();
    double dt = timestamp - lastUpdateTimestamp;
    lastUpdateTimestamp = timestamp;

    positionSim.update(dt);
    intakeSim.update(dt);

    double positionRadians = positionSim.getAngularPositionRad();
    double minRadians = Units.rotationsToRadians(IntakeConstants.kIntakeMinMechanismRotations);
    double maxRadians = Units.rotationsToRadians(IntakeConstants.kIntakeMaxMechanismRotations);

    if (positionRadians < minRadians) {
      positionRadians = minRadians;
      positionSim.setState(positionRadians, 0.0);
    } else if (positionRadians > maxRadians) {
      positionRadians = maxRadians;
      positionSim.setState(positionRadians, 0.0);
    }

    boolean atMin = positionRadians <= minRadians;
    boolean atMax = positionRadians >= maxRadians;
    double positionVelocityRadPerSec = positionSim.getAngularVelocityRadPerSec();
    boolean pushingMin = positionVoltage < 0.0;
    boolean pushingMax = positionVoltage > 0.0;

    if ((atMin && pushingMin) || (atMax && pushingMax)) {
      simulatedStall = true;
      positionVelocityRadPerSec = 0.0;
      positionSim.setState(positionRadians, 0.0);
    } else {
      simulatedStall = false;
    }

    double rotorPosition =
        Units.radiansToRotations(positionRadians) * IntakeConstants.kPositionGearRatio;
    double rotorVelocity =
        Units.radiansToRotations(positionVelocityRadPerSec) * IntakeConstants.kPositionGearRatio;
    positionSimState.setRawRotorPosition(rotorPosition);
    positionSimState.setRotorVelocity(rotorVelocity);

    intakeSimState.setRotorVelocity(
        Units.radiansToRotations(intakeSim.getAngularVelocityRadPerSec()));
  }
}
