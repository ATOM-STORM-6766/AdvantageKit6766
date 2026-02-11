package frc.robot.subsystems.flywheel;

import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.AutoLog;

public interface LimitSwitchIO {

  @AutoLog
  class LimitSwitchInputs {
    public boolean limitSwitch0 = false;
    public boolean limitSwitch1 = false;
    public boolean limitSwitch2 = false;
  }

  public void updateInputs(LimitSwitchInputs inputs);

  public BooleanSupplier getLimitSwitch0();

  public BooleanSupplier getLimitSwitch1();

  public BooleanSupplier getLimitSwitch2();
}
