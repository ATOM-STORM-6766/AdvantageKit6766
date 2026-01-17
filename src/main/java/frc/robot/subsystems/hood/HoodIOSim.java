package frc.robot.subsystems.hood;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import org.littletonrobotics.junction.Logger;

public class HoodIOSim extends HoodIOTalonFX {
  private static final DCMotor HOOD_MOTOR = DCMotor.getKrakenX60Foc(1);
  private static final double HOOD_MOMENT_OF_INERTIA = 0.1266;

  private static final double SIM_STALL_CURRENT_AMPS =
      HoodConstants.kCalibrationCurrentThreshold + 5.0;
  protected final DCMotorSim mechanismSim;
  private final Notifier simNotifier;
  private double lastUpdateTimestamp;
  private boolean simulatedStall = false;

  public HoodIOSim() {
    super();

    mechanismSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                HOOD_MOTOR, HOOD_MOMENT_OF_INERTIA, HoodConstants.kHoodGearRatio),
            HOOD_MOTOR);

    lastUpdateTimestamp = Timer.getFPGATimestamp();

    // 模拟初始位置是随机的
    mechanismSim.setState(Units.degreesToRadians(30), 0.0);

    simNotifier = new Notifier(this::updateSimState);
    simNotifier.startPeriodic(0.005);
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

  @Override
  public void readInputs(HoodInputs inputs) {
    super.readInputs(inputs);

    if (simulatedStall) {
      inputs.currentStatorAmps = SIM_STALL_CURRENT_AMPS;
      inputs.currentSupplyAmps = Math.max(inputs.currentSupplyAmps, SIM_STALL_CURRENT_AMPS);
    }
  }

  public void updateSimState() {
    var simState = hoodMotor.getSimState();
    simState.setSupplyVoltage(12.0);
    double motorVoltage = simState.getMotorVoltage();
    double simVoltage = addFriction(motorVoltage, 0.25);

    mechanismSim.setInput(simVoltage);
    Logger.recordOutput("Hood/Sim/SimulatorVoltage", simVoltage);

    double timestamp = Timer.getFPGATimestamp();
    mechanismSim.update(timestamp - lastUpdateTimestamp);
    lastUpdateTimestamp = timestamp;

    double simPositionRads = mechanismSim.getAngularPositionRad();
    double minAngle = HoodConstants.kHoodMinPositionRadians;
    double maxAngle = HoodConstants.kHoodMaxPositionRadians;

    if (simPositionRads < minAngle) {
      simPositionRads = minAngle;
      mechanismSim.setState(simPositionRads, 0.0);
    } else if (simPositionRads > maxAngle) {
      simPositionRads = maxAngle;
      mechanismSim.setState(simPositionRads, 0.0);
    }

    Logger.recordOutput("Hood/Sim/SimulatorPositionRadians", simPositionRads);

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

    double rotorPosition = Units.radiansToRotations(simPositionRads) * HoodConstants.kHoodGearRatio;
    simState.setRawRotorPosition(rotorPosition);
    Logger.recordOutput("Hood/Sim/setRawRotorPosition", rotorPosition);

    double rotorVel = Units.radiansToRotations(simVelocityRadPerSec) * HoodConstants.kHoodGearRatio;
    simState.setRotorVelocity(rotorVel);
    Logger.recordOutput("Hood/Sim/SimulatorVelocityRadS", simVelocityRadPerSec);
  }
}
