package frc.robot.subsystems.clamber;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Clamber extends SubsystemBase {
  private final ClamberIO io;
  private final ClamberInputsAutoLogged inputs = new ClamberInputsAutoLogged();

  public Clamber(ClamberIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Clamber", inputs);
  }

  /** Run the clamber motor at the specified voltage. */
  public void runVolts(double volts) {
    io.setVoltage(Volts.of(volts));
  }

  /** Run the clamber motor at the specified percentage. */
  public void runPercent(double percent) {
    runVolts(percent * 12.0);
  }

  /** Stop the clamber motor. */
  public void stop() {
    io.stop();
  }

  /** Command to run frame dependent on voltage */
  public Command runVoltsCommand(double volts) {
    return run(() -> runVolts(volts)).finallyDo(this::stop);
  }

  /** Command to run frame dependent on percent */
  public Command runPercentCommand(double percent) {
    return run(() -> runPercent(percent)).finallyDo(this::stop);
  }
}
