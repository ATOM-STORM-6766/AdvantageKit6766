package frc.robot.subsystems.light;

import com.ctre.phoenix6.signals.RGBWColor;
import com.ctre.phoenix6.signals.StatusLedWhenActiveValue;
import com.ctre.phoenix6.signals.StripTypeValue;
import com.ctre.phoenix6.configs.CANdleConfiguration;

public class LightConstants {
    public static final int candleCanID = 50;
    public static final int kCandleLightNum = 68;
    

    public static final RGBWColor kRed = new RGBWColor (255,0,0,0);
    public static final RGBWColor kGreen = new RGBWColor (0,255,0,0);
    public static final RGBWColor kBlue = new RGBWColor (0,0,255,0);
    public static final RGBWColor kWhite = new RGBWColor (0,0,0,255);

    public static RGBWColor getRGBWColor(String color){
        return switch (color) {
            case "red" -> kRed;
            case "green" -> kGreen;
            case "blue" -> kBlue;
            case "white" -> kWhite;
            default -> kGreen;
        };
    }

    public static RGBWColor onboardColor = kGreen;

    public static CANdleConfiguration getLightConfig(){
        var config = new CANdleConfiguration();
        config.LED.StripType = StripTypeValue.GRB;
        config.LED.BrightnessScalar = 1;
        config.CANdleFeatures.StatusLedWhenActive = StatusLedWhenActiveValue.Disabled;
        return config;
    }
}
