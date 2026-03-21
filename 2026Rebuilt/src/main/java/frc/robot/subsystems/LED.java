package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.controls.RainbowAnimation;
import com.ctre.phoenix6.controls.TwinkleAnimation;
import com.ctre.phoenix6.controls.FireAnimation;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.StripTypeValue;
import com.ctre.phoenix6.signals.RGBWColor;
import com.ctre.phoenix6.controls.ColorFlowAnimation;
import com.ctre.phoenix6.controls.LarsonAnimation;
import com.ctre.phoenix6.signals.LarsonBounceValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class LED extends SubsystemBase {
private static final int CANDLE_ID = 23;
private static final int LED_START = 0;
private static final int LED_COUNT = 100;
private int twinkleIndex = 0;

private final CANdle candle = new CANdle(CANDLE_ID);

public LED() {
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

public void setColorFlow(){
  RGBWColor colorFlowColor = new RGBWColor(255, 0, 0); //red
  candle.setControl(
    new ColorFlowAnimation(LED_START, LED_COUNT).withColor(colorFlowColor));
}

public void setTwinkle() {
  int ledEnd = LED_START + LED_COUNT - 1; // inclusive end index

  RGBWColor[] colors = {
    new RGBWColor(0, 255, 255), // Cyan
    new RGBWColor(0, 0, 255),   // Blue
    new RGBWColor(255, 0, 255), // Magenta
    new RGBWColor(128, 0, 128),  // Purple
    new RGBWColor(255,0,0)  // Red
  };

  RGBWColor color = colors[twinkleIndex];

  twinkleIndex = (twinkleIndex+1) % colors.length;

  candle.setControl(
    new TwinkleAnimation(LED_START, ledEnd)
      .withColor(color));
}

public void setLarson() {
  int ledEnd = LED_START + LED_COUNT - 1;

  candle.setControl(
    new LarsonAnimation(LED_START, ledEnd)
      .withColor(new RGBWColor(255, 0, 255)) //magenta
      .withSize(10)
      .withBounceMode(LarsonBounceValue.Front)
      .withFrameRate(120));
  }

  public void runColorFlowPattern(int r, int g, int b) {
    RGBWColor color = new RGBWColor(r, g, b);
    candle.setControl(new ColorFlowAnimation(LED_START, LED_COUNT).withColor(color));
  }

  public void runRainbow() {
    candle.setControl(new RainbowAnimation(LED_START, LED_COUNT));
  }

  public void runTwinkle(int r, int g, int b, int w) {
    candle.setControl(new TwinkleAnimation(LED_START, LED_COUNT));
  }

  public Command runFire() {
    return new InstantCommand(() -> candle.setControl(new FireAnimation(LED_START, LED_COUNT)));
  }
   
}
/*
 * Command runCommand = run(() -> pattern.applyTo(candle));
    runCommand.addRequirements(this);
    return runCommand;
 */