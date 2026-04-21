package frc.robot.subsystems.flywheel;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class FlywheelIOSim extends FlywheelIOTalonFX {
  private static final DCMotor FLYWHEEL_MOTOR = DCMotor.getKrakenX60Foc(4);
  private static final double FLYWHEEL_MOMENT_OF_INERTIA = 0.004;

  private final FlywheelSim sim0;
  private final Notifier simNotifier;
  private double lastUpdateTimestamp;

  public FlywheelIOSim() {
    super();

    var system =
        LinearSystemId.createFlywheelSystem(
            FLYWHEEL_MOTOR, FLYWHEEL_MOMENT_OF_INERTIA, FlywheelConstants.kFlywheelGearRatio);
    sim0 = new FlywheelSim(system, FLYWHEEL_MOTOR, FLYWHEEL_MOMENT_OF_INERTIA);

    lastUpdateTimestamp = Timer.getFPGATimestamp();
    simNotifier = new Notifier(this::updateSim);
    simNotifier.startPeriodic(0.005);
  }

  private void updateSim() {

    var motor0State = motor0.getSimState();
    var motor1State = motor1.getSimState();
    var motor2State = motor2.getSimState();
    var motor3State = motor3.getSimState();

    motor0State.setSupplyVoltage(12.0);
    motor1State.setSupplyVoltage(12.0);
    motor2State.setSupplyVoltage(12.0);
    motor3State.setSupplyVoltage(12.0);

    double leaderMotorVoltage = motor0State.getMotorVoltage();
    sim0.setInputVoltage(leaderMotorVoltage);

    motor0State.setRotorVelocity(
        sim0.getAngularVelocityRPM() / 60.0 * FlywheelConstants.kFlywheelGearRatio);
    motor1State.setRotorVelocity(
        sim0.getAngularVelocityRPM() / 60.0 * FlywheelConstants.kFlywheelGearRatio);
    motor2State.setRotorVelocity(
        sim0.getAngularVelocityRPM() / 60.0 * FlywheelConstants.kFlywheelGearRatio);
    motor3State.setRotorVelocity(
        sim0.getAngularVelocityRPM() / 60.0 * FlywheelConstants.kFlywheelGearRatio);

    double timestamp = Timer.getFPGATimestamp();
    double dt = timestamp - lastUpdateTimestamp;
    lastUpdateTimestamp = timestamp;
    sim0.update(dt);
  }
}
