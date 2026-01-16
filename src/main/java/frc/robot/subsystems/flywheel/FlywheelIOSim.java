package frc.robot.subsystems.flywheel;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class FlywheelIOSim extends FlywheelIOTalonFX {
  private static final DCMotor FLYWHEEL_MOTOR = DCMotor.getKrakenX60Foc(1);
  private static final double FLYWHEEL_MOMENT_OF_INERTIA = 0.004;
  private final FlywheelSim sim;
  private final Notifier simNotifier;
  private double lastUpdateTimestamp;
  private double appliedVolts = 0.0;

  public FlywheelIOSim() {
    super();

    // Simple flywheel simulation
    sim =
        new FlywheelSim(
            LinearSystemId.createFlywheelSystem(
                FLYWHEEL_MOTOR,
                FLYWHEEL_MOMENT_OF_INERTIA,
                FlywheelConstants.kFlywheelGearRatio),
            FLYWHEEL_MOTOR,
            FLYWHEEL_MOMENT_OF_INERTIA);

    lastUpdateTimestamp = Timer.getFPGATimestamp();
    simNotifier = new Notifier(this::updateSim);
    simNotifier.startPeriodic(0.005);
  }

  @Override
  public void readInputs(FlywheelInputs inputs) {
    inputs.velocityRPM = sim.getAngularVelocityRPM();
    inputs.appliedVolts = appliedVolts;
    inputs.currentStatorAmps = sim.getCurrentDrawAmps();
    inputs.currentSupplyAmps = sim.getCurrentDrawAmps();
  }

  @Override
  public void setVelocity(double velocityRPM, double ffVolts) {
    appliedVolts = ffVolts; // Simplified - just use FF voltage
    sim.setInputVoltage(appliedVolts);
  }

  private void updateSim() {
    double timestamp = Timer.getFPGATimestamp();
    sim.update(timestamp - lastUpdateTimestamp);
    lastUpdateTimestamp = timestamp;
  }
}
