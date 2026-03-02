package frc.robot.util;

import frc.robot.Constants;
import java.util.Arrays;
import java.util.HashMap;
import java.util.Map;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

public class LoggedTunableBoolean implements BooleanSupplier {
  private static final String tableKey = "/Tuning";

  private final String key;
  private boolean hasDefault = false;
  private boolean defaultValue;
  private LoggedNetworkBoolean dashboardBoolean;
  private final Map<Integer, Boolean> lastHasChangedValues = new HashMap<>();

  public LoggedTunableBoolean(String dashboardKey) {
    this.key = tableKey + "/" + dashboardKey;
  }

  public LoggedTunableBoolean(String dashboardKey, boolean defaultValue) {
    this(dashboardKey);
    initDefault(defaultValue);
  }

  public void initDefault(boolean defaultValue) {
    if (!hasDefault) {
      hasDefault = true;
      this.defaultValue = defaultValue;
      if (useNetworkTables()) {
        dashboardBoolean = new LoggedNetworkBoolean(key, defaultValue);
      }
    }
  }

  public boolean get() {
    if (!hasDefault) {
      return false;
    }
    if (useNetworkTables() && dashboardBoolean != null) {
      return dashboardBoolean.get();
    }
    return defaultValue;
  }

  public boolean hasChanged(int id) {
    boolean currentValue = get();
    Boolean lastValue = lastHasChangedValues.get(id);
    if (lastValue == null || currentValue != lastValue) {
      lastHasChangedValues.put(id, currentValue);
      return true;
    }
    return false;
  }

  public static void ifChanged(
      int id, Consumer<boolean[]> action, LoggedTunableBoolean... tunableBooleans) {
    if (Arrays.stream(tunableBooleans).anyMatch(tunableBoolean -> tunableBoolean.hasChanged(id))) {
      boolean[] values = new boolean[tunableBooleans.length];
      for (int i = 0; i < tunableBooleans.length; i++) {
        values[i] = tunableBooleans[i].get();
      }
      action.accept(values);
    }
  }

  public static void ifChanged(int id, Runnable action, LoggedTunableBoolean... tunableBooleans) {
    ifChanged(id, values -> action.run(), tunableBooleans);
  }

  @Override
  public boolean getAsBoolean() {
    return get();
  }

  private boolean useNetworkTables() {
    return Constants.currentMode != Constants.Mode.REPLAY;
  }
}
