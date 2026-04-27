package frc.robot.subsystems.light;

import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.RGBWColor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Light extends SubsystemBase {
  private final CANdle m_candle = new CANdle(LightConstants.candleCanID);
  // 状态枚举
  public enum LightState {
    ALLIANCE, // 显示联盟颜色
    AIM, // 瞄准状态
    READY, // 准备就绪状态
  }

  private LightState state = LightState.ALLIANCE;
  private LightState previousState = null;
  private DriverStation.Alliance alliance;
  private RGBWColor allianceColor;

  public Light() {
    m_candle.getConfigurator().apply(LightConstants.getLightConfig());
    /* clear all previous animation slots (CANdle supports 8 slots: 0-7) */
    for (int slot = 0; slot < 8; ++slot) {
      m_candle.setControl(new EmptyAnimation(slot));
    }
    m_candle.setControl(new SolidColor(0, 7).withColor(LightConstants.onboardColor));
    alliance = DriverStation.getAlliance().orElse(DriverStation.Alliance.Red);
    if (alliance == DriverStation.Alliance.Red) {
      allianceColor = LightConstants.kRed;
    } else {
      allianceColor = LightConstants.kBlue;
    }
  }

  // 设置左侧灯光颜色
  public void setLeftBarColor(RGBWColor color) {
    m_candle.setControl(new SolidColor(8, 37).withColor(color));
  }

  // 设置右侧灯光颜色
  public void setRightBarColor(RGBWColor color) {
    m_candle.setControl(new SolidColor(38, 67).withColor(color));
  }

  // 设置灯条整体颜色的命令
  public Command setLightColor(String color) {
    return Commands.runOnce(
        () -> {
          RGBWColor c = LightConstants.getRGBWColor(color);
          setLeftBarColor(c);
          setRightBarColor(c);
        });
  }

  public void setState(LightState state) {
    this.state = state;
  }

  public Command setStateCommand(LightState state) {
    return Commands.runOnce(() -> setState(state));
  }

  @Override
  public void periodic() {
    // Publish current logical state and alliance to SmartDashboard for the sim/UI
    SmartDashboard.putString("Lights/State", state.name());
    SmartDashboard.putString(
        "Lights/Alliance", DriverStation.getAlliance().orElse(DriverStation.Alliance.Red).name());
    if (state != previousState) {
      switch (state) {
        case ALLIANCE -> {
          setLeftBarColor(allianceColor);
          setRightBarColor(allianceColor);
        }
        case AIM -> {
          setLeftBarColor(LightConstants.kMagenta);
          setRightBarColor(LightConstants.kMagenta);
        }
        case READY -> {
          setLeftBarColor(LightConstants.kGreen);
          setRightBarColor(LightConstants.kGreen);
        }
      }
      previousState = state;
    }
  }
}
