package frc.robot.subsystems.flywheel;

import edu.wpi.first.wpilibj.DigitalInput;
import java.util.function.BooleanSupplier;

public class LimitSwitchDIO implements LimitSwitchIO {

  private final DigitalInput digitalInput0;
  private final DigitalInput digitalInput1;
  private final DigitalInput digitalInput2;

  public LimitSwitchDIO(int dioPort0, int dioPort1, int dioPort2) {
    digitalInput0 = new DigitalInput(dioPort0);
    digitalInput1 = new DigitalInput(dioPort1);
    digitalInput2 = new DigitalInput(dioPort2);
  }

  @Override
  public void updateInputs(LimitSwitchInputs inputs) {
    inputs.limitSwitch0 = digitalInput0.get();
    inputs.limitSwitch1 = digitalInput1.get();
    inputs.limitSwitch2 = digitalInput2.get();
  }

  @Override
  public BooleanSupplier getLimitSwitch0() {
    return digitalInput0::get;
  }

  @Override
  public BooleanSupplier getLimitSwitch1() {
    return digitalInput1::get;
  }

  @Override
  public BooleanSupplier getLimitSwitch2() {
    return digitalInput2::get;
  }
}
