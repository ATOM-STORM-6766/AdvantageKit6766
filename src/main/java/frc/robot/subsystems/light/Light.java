package frc.robot.subsystems.light;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.RGBWColor;

public class Light extends SubsystemBase{
    private final CANdle candle = new CANdle(LightConstants.candleCanID);
    //状态枚举
    public enum LightState{
        ALLIANCE, //显示联盟颜色
        IDLE, //空载颜色
        INTAKE, //吸球状态
        SHOOTING, //射球状态
        AIM, //瞄准状态
        ERROR, //错误状态

    }

    private volatile LightState state = LightState.ALLIANCE;
    private LightState lastAppliedState = null;
    private DriverStation.Alliance lastAppliedAlliance = null;

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

    public void setState(LightState state){
        this.state = state;
        this.lastAppliedState = null; 
    }

    public Command setStateCommand(LightState state){
        return Commands.runOnce(() -> setState(state),this);
    }
    
    @Override
    public void periodic(){
        DriverStation.Alliance alliance = DriverStation.getAlliance().get();
        if (state == lastAppliedState && (state != LightState.ALLIANCE || alliance == lastAppliedAlliance)) {
            return;
        }
        RGBWColor apply;
        switch (state) {
            case ALLIANCE:
                if (alliance == DriverStation.Alliance.Red) {
                    apply = LightConstants.kRed;
                } else if (alliance == DriverStation.Alliance.Blue) {
                    apply = LightConstants.kBlue;
                } else {
                    apply = LightConstants.onboardColor;
                }
                break;
            case INTAKE:
                apply = LightConstants.kGreen;
                break;
            case SHOOTING:
                apply = LightConstants.kYellow;
                break;
            case AIM:
                apply = LightConstants.kCyan;
                break;
            case ERROR:
                candle.setControl(
                new StrobeAnimation(8, 67)
                    .withColor(LightConstants.kRed)
                );
                lastAppliedState = state;
                lastAppliedAlliance = alliance;
                return;
            case IDLE:
            default:
                apply = LightConstants.defaultColor;
                break;
        }
        lastAppliedState = state;
        lastAppliedAlliance = alliance;
        setLeftBarColor(apply);
        setRightBarColor(apply);
    }

}
