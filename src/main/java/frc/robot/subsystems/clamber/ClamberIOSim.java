package frc.robot.subsystems.clamber;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

public class ClamberIOSim extends ClamberIOTalonFX {
  private final ElevatorSim sim;
  private final Notifier simNotifier;

  public ClamberIOSim() {
    super();

    sim =
        new ElevatorSim(
            DCMotor.getKrakenX60Foc(1),
            ClamberConstants.kGearRatio,
            10.0, // Carriage mass kg
            0.02, // Drum radius m
            0.0, // Min height m
            2.0, // Max height m
            true, // Simulate gravity
            0.0 // Starting height
            );

    simNotifier = new Notifier(this::updateSim);
    simNotifier.startPeriodic(0.02);
  }

  private void updateSim() {
    var motorSim = motor.getSimState();
    motorSim.setSupplyVoltage(RobotController.getBatteryVoltage());

    sim.setInputVoltage(motorSim.getMotorVoltage());
    sim.update(0.02);

    motorSim.setRawRotorPosition(
        sim.getPositionMeters() / (2 * Math.PI * 0.02) * ClamberConstants.kGearRatio);
    motorSim.setRotorVelocity(
        sim.getVelocityMetersPerSecond() / (2 * Math.PI * 0.02) * ClamberConstants.kGearRatio);
  }
}
