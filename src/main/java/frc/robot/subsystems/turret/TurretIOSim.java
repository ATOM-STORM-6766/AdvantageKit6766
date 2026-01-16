package frc.robot.subsystems.turret;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import org.littletonrobotics.junction.Logger;

public class TurretIOSim extends TurretIOTalonFX {
  private static final DCMotor TURRET_MOTOR = DCMotor.getKrakenX60Foc(1);
  private static final double TURRET_MOMENT_OF_INERTIA = 0.2981858;
  private static final double CALIBRATION_DELAY_SECONDS = 2.0;
  protected final DCMotorSim mechanismSim;
  protected double lastUpdateTimestamp;
  private final Notifier simNotifier;
  private final Timer calibrationTimer = new Timer();

  public TurretIOSim() {
    super();

    mechanismSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                TURRET_MOTOR, TURRET_MOMENT_OF_INERTIA, TurretConstants.kTurretGearRatio),
            TURRET_MOTOR);

    lastUpdateTimestamp = Timer.getFPGATimestamp();

    // Initialize DCMotorSim state to zero position and velocity
    mechanismSim.setState(0.0, 0.0);

    // Initialize TalonFX simulation state
    talon.getSimState().setRawRotorPosition(0.0);

    // Run simulation at a faster rate so PID gains behave more reasonably
    simNotifier = new Notifier(this::updateSimState);
    simNotifier.startPeriodic(0.005);
  }

  @Override
  public void startCalibration() {
    // Simplified calibration for simulation
    // Set position to min limit and start calibration timer
    double minPositionRad = TurretConstants.kTurretMinPositionRadians;

    // Update simulator state
    mechanismSim.setState(minPositionRad, 0.0);

    // Update TalonFX sim state
    double positionRotations = Units.radiansToRotations(minPositionRad);
    double positionRotor = positionRotations * TurretConstants.kTurretGearRatio;
    talon.getSimState().setRawRotorPosition(positionRotor);
    talon.getSimState().setRotorVelocity(0.0);

    // Start calibration timer and set state to CALIBRATING
    calibrationTimer.restart();
    calibrationState = CalibrationState.CALIBRATING;
  }

  @Override
  public void readInputs(TurretInputs inputs) {
    // Check if calibration is complete after delay
    if (calibrationState == CalibrationState.CALIBRATING
        && calibrationTimer.hasElapsed(CALIBRATION_DELAY_SECONDS)) {
      calibrationState = CalibrationState.CALIBRATED;
      calibrationTimer.stop();
    }

    // Call parent implementation
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
    // Step sim
    var simState = talon.getSimState();
    simState.setSupplyVoltage(12.0);
    double simVoltage = addFriction(simState.getMotorVoltage(), 0.25);

    mechanismSim.setInput(simVoltage);
    Logger.recordOutput("Turret/Sim/SimulatorVoltage", simVoltage);

    double timestamp = Timer.getFPGATimestamp();
    mechanismSim.update(timestamp - lastUpdateTimestamp);
    lastUpdateTimestamp = timestamp;

    // Find current state of sim in radians from 0 point
    double simPositionRads = mechanismSim.getAngularPositionRad();
    double minAngle = TurretConstants.kTurretMinPositionRadians;
    double maxAngle = TurretConstants.kTurretMaxPositionRadians;

    // Clamp to physical limits for limited rotation range
    if (simPositionRads < minAngle) {
      simPositionRads = minAngle;
      mechanismSim.setState(simPositionRads, 0.0);
    } else if (simPositionRads > maxAngle) {
      simPositionRads = maxAngle;
      mechanismSim.setState(simPositionRads, 0.0);
    }

    Logger.recordOutput("Turret/Sim/SimulatorPositionRadians", simPositionRads);

    double simVelocityRadPerSec = mechanismSim.getAngularVelocityRadPerSec();
    if (simPositionRads <= minAngle || simPositionRads >= maxAngle) {
      simVelocityRadPerSec = 0.0;
    }

    // Mutate rotor position
    double rotorPosition =
        Units.radiansToRotations(simPositionRads) * TurretConstants.kTurretGearRatio;
    simState.setRawRotorPosition(rotorPosition);
    Logger.recordOutput("Turret/Sim/setRawRotorPosition", rotorPosition);

    // Mutate rotor vel
    double rotorVel =
        Units.radiansToRotations(simVelocityRadPerSec) * TurretConstants.kTurretGearRatio;
    simState.setRotorVelocity(rotorVel);
    Logger.recordOutput("Turret/Sim/SimulatorVelocityRadS", simVelocityRadPerSec);
  }
}
