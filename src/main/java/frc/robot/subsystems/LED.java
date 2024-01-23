package frc.robot.subsystems;
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

import java.util.ArrayList;
import java.util.Arrays;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LED extends SubsystemBase {

  private static ArrayList<Double> wiperEffect = new ArrayList<>(Arrays.asList(
    0.1, 0.5, 1.0, 1.0, 1.0, 1.0, 0.5, 0.1
  ));

  private static ArrayList<Double> rainEffect = new ArrayList<>(Arrays.asList(
    0.25, 1.0
  ));
  
  private static LED m_instance = null;

  private static AddressableLED m_led;

  private static AddressableLEDBuffer m_ledBuffer;

  private static LEDState m_currentState = LEDState.OFF;

  private static int m_rainbowFirstPixelHue = 0;

  private static boolean hasEffect = false;

  public LED(AddressableLED led, int length) {
    m_led = led;

    m_ledBuffer = new AddressableLEDBuffer(length);
    m_led.setLength(length);

    m_led.setData(m_ledBuffer);
    m_led.start();
  }

  public static LED getInstance() {
    if (m_instance == null) {
      m_instance = new LED(new AddressableLED(0), 300);
    }

    return m_instance;
  }

  public void setState(LEDState desiredState) {
    m_currentState = desiredState;
    if (desiredState.ledRunnable == null) {
      setRGB(desiredState.red, desiredState.green, desiredState.blue);
    }
  }

  public void setRGB(int r, int g, int b) {
    for (int i = 0; i < m_ledBuffer.getLength(); i++) {
      m_ledBuffer.setRGB(i, r, g, b);
    }

    m_led.setData(m_ledBuffer);
  }

  public LEDState getCurrentState() {
    return m_currentState;
  }

  public static void runEffect(ArrayList<Double> effect) {
    for (int i = 0; i < effect.size(); i++) {
      m_ledBuffer.setRGB(i, (int) (m_currentState.red * effect.get(i)), (int) (m_currentState.green * effect.get(i)), (int) (m_currentState.blue * effect.get(i)));
    }
    effect.add(0, effect.remove(effect.size() - 1));

    m_led.setData(m_ledBuffer);
    // System.out.println(effect.toString());
  }

  public static void rainbow() {
    // For every pixel
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      // Calculate the hue - hue is easier for rainbows because the color
      // shape is a circle so only one value needs to precess
      final var hue = (m_rainbowFirstPixelHue + (i * 180 / m_ledBuffer.getLength())) % 180;
      // Set the value
      m_ledBuffer.setHSV(i, hue, 255, 128);
    }
    // Increase by to make the rainbow "move"
    m_rainbowFirstPixelHue += 3;
    // Check bounds
    m_rainbowFirstPixelHue %= 180;
    m_led.setData(m_ledBuffer);
  }

  @Override
  public void periodic() {
    if (m_currentState.ledRunnable != null) {
      m_currentState.ledRunnable.run();
      if (!hasEffect) hasEffect = true;
    } else if (hasEffect) {
      setState(m_currentState);
      hasEffect = false;
    }
  }

  public enum LEDState {
    OFF(0, 0, 0, "Off", null),
    BLUE(0, 0, 255, "Blue", null),
    RED(255, 0, 0, "Red", null),
    GREEN(0, 255, 0, "Green", null),
    RAINBOW(0, 0, 0, "Rainbow", LED::rainbow),
    TEAL_WIPE(0, 122, 133, "Teal Wipe", () -> LED.runEffect(wiperEffect)),
    TEAL_RAIN(0, 122, 133, "Teal Wipe", () -> LED.runEffect(rainEffect));

    public int red;
    public int green;
    public int blue;
    public Runnable ledRunnable;
    public String name;

    private LEDState(int red, int green, int blue, String name, Runnable ledRunnable) {
      this.red = red;
      this.green = green;
      this.blue = blue;
      this.ledRunnable = ledRunnable;
      this.name = name;
    }
  }

}
