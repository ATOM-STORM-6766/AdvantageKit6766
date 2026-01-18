package frc.robot.subsystems.turret;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.DutyCycleEncoderSim;

public class TurretIOSim extends TurretIOTalonFX {
  private static final DCMotor TURRET_MOTOR = DCMotor.getKrakenX60Foc(1);
  private static final double TURRET_MOMENT_OF_INERTIA = 0.0181858;

  private static final double SIM_STALL_CURRENT_AMPS =
      TurretConstants.kCalibrationCurrentThreshold + 5.0;
  protected final DCMotorSim mechanismSim;
  protected final DutyCycleEncoderSim absoluteEncoderSim;
  protected final DutyCycleEncoderSim absoluteEncoder2Sim;

  private final Notifier simNotifier;
  protected double lastUpdateTimestamp;
  private boolean simulatedStall = false;

  public TurretIOSim() {
    super();

    mechanismSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                TURRET_MOTOR, TURRET_MOMENT_OF_INERTIA, TurretConstants.kTurretGearRatio),
            TURRET_MOTOR);

    absoluteEncoderSim = new DutyCycleEncoderSim(absoluteEncoder);
    absoluteEncoder2Sim = new DutyCycleEncoderSim(absoluteEncoder2);

    lastUpdateTimestamp = Timer.getFPGATimestamp();

    // 模拟初始位置是随机的
    double randomInitialTurretPosition = 30;
    mechanismSim.setState(Units.degreesToRadians(randomInitialTurretPosition), 0.0);
    double encoder1Position =
        Units.degreesToRotations(randomInitialTurretPosition) * TurretConstants.kTurretAbsoluteEncoderToTurretRatio;
    double encoder2Position =
        Units.degreesToRotations(randomInitialTurretPosition) * TurretConstants.kTurretAbsoluteEncoder2ToTurretRatio;
    absoluteEncoderSim.set(encoder1Position - Math.floor(encoder1Position));
    absoluteEncoder2Sim.set(encoder2Position - Math.floor(encoder2Position));

    simNotifier = new Notifier(this::updateSimState);
    simNotifier.startPeriodic(0.005);
  }

  @Override
  public void readInputs(TurretInputs inputs) {
    super.readInputs(inputs);

    if (simulatedStall) {
      inputs.currentStatorAmps = SIM_STALL_CURRENT_AMPS;
      inputs.currentSupplyAmps = Math.max(inputs.currentSupplyAmps, SIM_STALL_CURRENT_AMPS);
    }
  }

  protected double addFriction(double motorVoltage, double frictionVoltage) {
    if (Math.abs(motorVoltage) < frictionVoltage) {
      return 0.0;
    } else if (motorVoltage > 0.0) {
      return motorVoltage - frictionVoltage;
    } else {
      return motorVoltage + frictionVoltage;
    }
  }

  public void updateSimState() {
    var simState = talon.getSimState();

    simState.setSupplyVoltage(12.0);
    double motorVoltage = simState.getMotorVoltage();
    double simVoltage = addFriction(motorVoltage, 0.25);

    mechanismSim.setInput(simVoltage);

    double timestamp = Timer.getFPGATimestamp();
    mechanismSim.update(timestamp - lastUpdateTimestamp);
    lastUpdateTimestamp = timestamp;

    double simPositionRads = mechanismSim.getAngularPositionRad();
    double minAngle = TurretConstants.kTurretMinPositionRadians;
    double maxAngle = TurretConstants.kTurretMaxPositionRadians;

    if (simPositionRads < minAngle) {
      simPositionRads = minAngle;
      mechanismSim.setState(simPositionRads, 0.0);
    } else if (simPositionRads > maxAngle) {
      simPositionRads = maxAngle;
      mechanismSim.setState(simPositionRads, 0.0);
    }

    boolean atMin = simPositionRads <= minAngle;
    boolean atMax = simPositionRads >= maxAngle;
    double simVelocityRadPerSec = mechanismSim.getAngularVelocityRadPerSec();

    boolean pushingMin = simVoltage < 0;
    boolean pushingMax = simVoltage > 0;

    if ((atMin && pushingMin) || (atMax && pushingMax)) {
      simulatedStall = true;
      simState.setRotorVelocity(0.0);
    } else {
      simulatedStall = false;
      simState.setRotorVelocity(0.0);
    }

    double rotorPosition =
        Units.radiansToRotations(simPositionRads) * TurretConstants.kTurretGearRatio;
    simState.setRawRotorPosition(rotorPosition);

    double rotorVel =
        Units.radiansToRotations(simVelocityRadPerSec) * TurretConstants.kTurretGearRatio;
    simState.setRotorVelocity(rotorVel);

    double turretRotations = Units.radiansToRotations(simPositionRads);

    // Absolute Encoder 1
    double absoluteEncoderRotations =
        turretRotations * TurretConstants.kTurretAbsoluteEncoderToTurretRatio;
    double absoluteEncoderPosition =
        absoluteEncoderRotations - Math.floor(absoluteEncoderRotations);
    absoluteEncoderSim.set(absoluteEncoderPosition);

    // Absolute Encoder 2
    double absoluteEncoder2Rotations =
        turretRotations * TurretConstants.kTurretAbsoluteEncoder2ToTurretRatio;
    double absoluteEncoder2Position =
        absoluteEncoder2Rotations - Math.floor(absoluteEncoder2Rotations);
    absoluteEncoder2Sim.set(absoluteEncoder2Position);
  }
}
