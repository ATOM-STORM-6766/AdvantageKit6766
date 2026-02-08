package frc.robot.subsystems.flywheel;

import org.littletonrobotics.junction.AutoLog;

public interface LimitSwitchIO {

  @AutoLog
  class LimitSwitchInputs {
    public boolean limitSwitch0 = false;
    public boolean limitSwitch1 = false;
    public boolean limitSwitch2 = false;
  }

  void updateInputs(LimitSwitchInputs inputs);
}
