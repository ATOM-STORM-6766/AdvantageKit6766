package frc.robot.subsystems.light;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.RGBWColor;


public class Light extends SubsystemBase{
    private final CANdle candle = new CANdle(LightConstants.candleCanID);
    
    public Light(){
        candle.getConfigurator().apply(LightConstants.getLightConfig());
        for (int i = 0; i < LightConstants.kCandleLightNum; ++i) {
            candle.setControl(new EmptyAnimation(i));
        }
        candle.setControl(new SolidColor(0,7).withColor(LightConstants.onboardColor));
    }

    public void setLeftBarColor(RGBWColor color){
        candle.setControl(new SolidColor(8,37).withColor(color));
    }

    public void setRightBarColor(RGBWColor color){
        candle.setControl(new SolidColor(38,67).withColor(color));
    }


    public Command setLightColor(String color){
        return Commands.runOnce(() -> {
            RGBWColor c = LightConstants.getRGBWColor(color);
            setLeftBarColor(c);
            setRightBarColor(c);
        }, this);
    }
}
