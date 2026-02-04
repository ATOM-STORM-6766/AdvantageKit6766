package frc.robot.subsystems.flywheel;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class FlywheelIOSim extends FlywheelIOTalonFX {
  private static final DCMotor FLYWHEEL_MOTOR = DCMotor.getKrakenX60Foc(1);
  private static final double FLYWHEEL_MOMENT_OF_INERTIA = 0.004;

  private final FlywheelSim sim0, sim1, sim2;
  private final Notifier simNotifier;
  private double lastUpdateTimestamp;
  private double appliedVolts0, appliedVolts1, appliedVolts2 = 0.0;

  public FlywheelIOSim() {
    super();

    var system =
        LinearSystemId.createFlywheelSystem(
            FLYWHEEL_MOTOR, FLYWHEEL_MOMENT_OF_INERTIA, FlywheelConstants.kFlywheelGearRatio);
    sim0 = new FlywheelSim(system, FLYWHEEL_MOTOR, FLYWHEEL_MOMENT_OF_INERTIA);
    sim1 = new FlywheelSim(system, FLYWHEEL_MOTOR, FLYWHEEL_MOMENT_OF_INERTIA);
    sim2 = new FlywheelSim(system, FLYWHEEL_MOTOR, FLYWHEEL_MOMENT_OF_INERTIA);

    lastUpdateTimestamp = Timer.getFPGATimestamp();
    simNotifier = new Notifier(this::updateSim);
    simNotifier.startPeriodic(0.005);
  }

  @Override
  public void readInputs(FlywheelInputs inputs) {
    inputs.velocityRps0 = sim0.getAngularVelocityRPM() / 60.0;
    inputs.appliedVolts0 = appliedVolts0;
    inputs.currentStatorAmps0 = sim0.getCurrentDrawAmps();
    inputs.currentSupplyAmps0 = sim0.getCurrentDrawAmps();

    inputs.velocityRps1 = sim1.getAngularVelocityRPM() / 60.0;
    inputs.appliedVolts1 = appliedVolts1;
    inputs.currentStatorAmps1 = sim1.getCurrentDrawAmps();
    inputs.currentSupplyAmps1 = sim1.getCurrentDrawAmps();

    inputs.velocityRps2 = sim2.getAngularVelocityRPM() / 60.0;
    inputs.appliedVolts2 = appliedVolts2;
    inputs.currentStatorAmps2 = sim2.getCurrentDrawAmps();
    inputs.currentSupplyAmps2 = sim2.getCurrentDrawAmps();
  }

  @Override
  public void setVelocity(double rps0, double rps1, double rps2) {
    double kV = FlywheelConstants.getTalonFXConfig().Slot0.kV;
    appliedVolts0 = kV * rps0;
    appliedVolts1 = kV * rps1;
    appliedVolts2 = kV * rps2;
    sim0.setInputVoltage(appliedVolts0);
    sim1.setInputVoltage(appliedVolts1);
    sim2.setInputVoltage(appliedVolts2);
  }

  private void updateSim() {
    double timestamp = Timer.getFPGATimestamp();
    double dt = timestamp - lastUpdateTimestamp;
    lastUpdateTimestamp = timestamp;
    sim0.update(dt);
    sim1.update(dt);
    sim2.update(dt);
  }
}
