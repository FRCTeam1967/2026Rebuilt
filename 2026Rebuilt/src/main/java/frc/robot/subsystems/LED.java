package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.controls.RainbowAnimation;
import com.ctre.phoenix6.controls.SolidColor;
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
}

/**
 * sets the LED to rainbow
 */
public void setRainbow() {
  candle.setControl(new RainbowAnimation(LED_START, LED_COUNT));
}

/**
   * sets the LED to black
*/
public void setBlack(){
  candle.setControl(new SolidColor(LED_START, LED_COUNT));
}

/**
   * sets the LED to the fire pattern (red, orange, and yellow)
*/
public void setFire() {
  candle.setControl(new FireAnimation(LED_START, LED_COUNT));
}

/**
   * sets the LED to red
*/
public void setColorFlow(){
  RGBWColor colorFlowColor = new RGBWColor(255, 0, 0);
  candle.setControl(
    new ColorFlowAnimation(LED_START, LED_COUNT).withColor(colorFlowColor));
}
/**
   * sets the LED to twinkle pattern
*/
public void setTwinkle() {
  int ledEnd = LED_START + LED_COUNT - 1;

  RGBWColor[] colors = {
    new RGBWColor(0, 255, 255), // Cyan
    new RGBWColor(0, 0, 255),   // Blue
    new RGBWColor(255, 0, 255), // Magenta
    new RGBWColor(128, 0, 128),  // Purple
    new RGBWColor(255,0,0),  // Red
    new RGBWColor(0, 0, 0) //Black
  };

  RGBWColor color = colors[twinkleIndex];

  twinkleIndex = (twinkleIndex+1) % colors.length;

  candle.setControl(
    new TwinkleAnimation(LED_START, ledEnd)
      .withColor(color));
}
/**
 * sets LEDs to a Larson Animation (scrolling one color)
 */
public void setLarson() {
  int ledEnd = LED_START + LED_COUNT - 1;

  candle.setControl(
    new LarsonAnimation(LED_START, ledEnd)
      .withColor(new RGBWColor(255, 0, 255)) //magenta
      .withSize(10)
      .withBounceMode(LarsonBounceValue.Front)
      .withFrameRate(120));
  }
  /**
    * @param r red RGB value in RGB 
    * @param g green RGB value in RBG
    * @param b blue RGB value in RGB
    */
  public void runColorFlowPattern(int r, int g, int b) {
    RGBWColor color = new RGBWColor(r, g, b);
    candle.setControl(new ColorFlowAnimation(LED_START, LED_COUNT).withColor(color));
  }
  /**
    * runs the rainbow scroll pattern
    */
  public void runRainbow() {
    candle.setControl(new RainbowAnimation(LED_START, LED_COUNT));
  }
  /**
   * @param r Red Value in RGB
   * @param g Green Value in RGB
   * @param b Blue Value in RGB
   * @param w Controls White value(brightness)
   * Sets the LEDs to Twinkle
   */
  public void runTwinkle(int r, int g, int b, int w) {
    candle.setControl(new TwinkleAnimation(LED_START, LED_COUNT));
  }
  /**
   * Creates a command that does runFire
   * @return the fire pattern (red, orange, and yellow) on the LED strip
*/
  public Command runFire() {
    return new InstantCommand(() -> candle.setControl(new FireAnimation(LED_START, LED_COUNT)));
  }
   
}
/*
 * Command runCommand = run(() -> pattern.applyTo(candle));
    runCommand.addRequirements(this);
    return runCommand;
 */