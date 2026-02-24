package frc.robot.subsystems.light;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.RGBWColor;

import edu.wpi.first.wpilibj.DriverStation;

public class Light extends SubsystemBase{
    private final CANdle candle = new CANdle(LightConstants.candleCanID);
    
    public Light(){
        candle.getConfigurator().apply(LightConstants.getLightConfig());
        for (int i = 0; i < LightConstants.kCandleLightNum; ++i) {
            candle.setControl(new EmptyAnimation(i));
        }
        candle.setControl(new SolidColor(0,7).withColor(LightConstants.onboardColor));
        // 根据联盟设置灯光颜色
        candle.setControl(new SolidColor(8,67).withColor(LightConstants.defaultColor));        
    }

    // 设置左侧灯光颜色
    public void setLeftBarColor(RGBWColor color){
        candle.setControl(new SolidColor(8,37).withColor(color));
    }

    // 设置右侧灯光颜色
    public void setRightBarColor(RGBWColor color){
        candle.setControl(new SolidColor(38,67).withColor(color));
    }

    // 设置灯条整体颜色的命令
    public Command setLightColor(String color){
        return Commands.runOnce(() -> {
            RGBWColor c = LightConstants.getRGBWColor(color);
            setLeftBarColor(c);
            setRightBarColor(c);
        }, this);
    }
}
