package frc.robot.subsystems.flywheel;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.DigitalInput;
import java.util.function.BooleanSupplier;

public class LimitSwitchDIO implements LimitSwitchIO {

  private final DigitalInput digitalInput0;
  private final DigitalInput digitalInput1;
  private final DigitalInput digitalInput2;
  private final Debouncer boostHoldDebouncer0 =
      new Debouncer(FlywheelConstants.kFlywheelBoostHoldTimeSec, Debouncer.DebounceType.kFalling);
  private final Debouncer boostHoldDebouncer1 =
      new Debouncer(FlywheelConstants.kFlywheelBoostHoldTimeSec, Debouncer.DebounceType.kFalling);
  private final Debouncer boostHoldDebouncer2 =
      new Debouncer(FlywheelConstants.kFlywheelBoostHoldTimeSec, Debouncer.DebounceType.kFalling);

  public LimitSwitchDIO() {
    digitalInput0 = new DigitalInput(FlywheelConstants.kLimitSwitchID0);
    digitalInput1 = new DigitalInput(FlywheelConstants.kLimitSwitchID1);
    digitalInput2 = new DigitalInput(FlywheelConstants.kLimitSwitchID2);
  }

  @Override
  public void updateInputs(LimitSwitchInputs inputs) {
    inputs.limitSwitch0 = boostHoldDebouncer0.calculate(digitalInput0.get());
    inputs.limitSwitch1 = boostHoldDebouncer1.calculate(digitalInput1.get());
    inputs.limitSwitch2 = boostHoldDebouncer2.calculate(digitalInput2.get());
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
