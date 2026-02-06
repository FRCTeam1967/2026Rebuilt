package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.controls.RainbowAnimation;
import com.ctre.phoenix6.controls.StrobeAnimation;
import com.ctre.phoenix6.controls.FireAnimation;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.StripTypeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase {
  private static final int CANDLE_ID = 0;
  private static final int LED_START = 8;        // 0-7 onboard, 8+ external strip
  private static final int LED_COUNT = 64;

  private final CANdle candle = new CANdle(CANDLE_ID);

  public LEDSubsystem() {
    CANdleConfiguration cfg = new CANdleConfiguration();
    cfg.LED.StripType = StripTypeValue.RGB;   
    cfg.LED.BrightnessScalar = 0.5;
    candle.getConfigurator().apply(cfg);

    setRainbow();
  }

  public void setRainbow() {
    candle.setControl(new RainbowAnimation(LED_START, LED_COUNT));
  }

  public void setFire() {
    candle.setControl(new FireAnimation(LED_START, LED_COUNT));
  }
}
