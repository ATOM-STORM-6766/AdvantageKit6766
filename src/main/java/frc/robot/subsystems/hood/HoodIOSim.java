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
  private static final double HOOD_MOMENT_OF_INERTIA = 0.01;
  protected final DCMotorSim mechanismSim;
  private final Notifier simNotifier;
  private double lastUpdateTimestamp;

  public HoodIOSim() {
    super();

    mechanismSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                HOOD_MOTOR, HOOD_MOMENT_OF_INERTIA, 1.0 / HoodConstants.kHoodGearRatio),
            HOOD_MOTOR);

    lastUpdateTimestamp = Timer.getFPGATimestamp();

    // Initialize DCMotorSim state to zero position and velocity
    mechanismSim.setState(0.0, 0.0);

    // Initialize TalonFX simulation state
    hoodMotor.getSimState().setRawRotorPosition(0.0);

    simNotifier = new Notifier(this::updateSimState);
    simNotifier.startPeriodic(0.005);
  }

  @Override
  public void readInputs(HoodInputs inputs) {
    super.readInputs(inputs);
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
    var simState = hoodMotor.getSimState();
    simState.setSupplyVoltage(12.0);
    double simVoltage = addFriction(simState.getMotorVoltage(), 0.25);

    mechanismSim.setInput(simVoltage);
    Logger.recordOutput("Hood/Sim/SimulatorVoltage", simVoltage);

    double timestamp = Timer.getFPGATimestamp();
    mechanismSim.update(timestamp - lastUpdateTimestamp);
    lastUpdateTimestamp = timestamp;

    double simPositionRads = mechanismSim.getAngularPositionRad();
    double minAngle = HoodConstants.kHoodMinPositionRadians;
    double maxAngle = HoodConstants.kHoodMaxPositionRadians;

    // Clamp to physical limits for limited rotation range
    if (simPositionRads < minAngle) {
      simPositionRads = minAngle;
      mechanismSim.setState(simPositionRads, 0.0);
    } else if (simPositionRads > maxAngle) {
      simPositionRads = maxAngle;
      mechanismSim.setState(simPositionRads, 0.0);
    }

    Logger.recordOutput("Hood/Sim/SimulatorPositionRadians", simPositionRads);

    double simVelocityRadPerSec = mechanismSim.getAngularVelocityRadPerSec();
    if (simPositionRads <= minAngle || simPositionRads >= maxAngle) {
      simVelocityRadPerSec = 0.0;
    }

    double rotorPosition = Units.radiansToRotations(simPositionRads) / HoodConstants.kHoodGearRatio;
    simState.setRawRotorPosition(rotorPosition);
    Logger.recordOutput("Hood/Sim/setRawRotorPosition", rotorPosition);

    double rotorVel = Units.radiansToRotations(simVelocityRadPerSec) / HoodConstants.kHoodGearRatio;
    simState.setRotorVelocity(rotorVel);
    Logger.recordOutput("Hood/Sim/SimulatorVelocityRadS", simVelocityRadPerSec);
  }
}
