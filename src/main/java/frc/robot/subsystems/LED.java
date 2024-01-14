package frc.robot.subsystems;
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LED extends SubsystemBase {
  /** Creates a new LED. */
  private AddressableLED m_led;

  private AddressableLEDBuffer m_ledBuffer;

  private LEDState m_currentState;

  private int m_rainbowFirstPixelHue = 0;

  private boolean rainbow = false;

  public LED(AddressableLED led, int length) {
    m_led = led;

    m_ledBuffer = new AddressableLEDBuffer(length);
    m_led.setLength(length);

    m_led.setData(m_ledBuffer);
    m_led.start();
  }

  public void setState(LEDState desiredState) {
    m_currentState = desiredState;
    if (desiredState == LEDState.RAINBOW) {
      rainbow = true;
    } else {
      rainbow = false;
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

  private void rainbow() {
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
    if (rainbow) rainbow();
  }

  public enum LEDState {
    BLUE(0, 0, 255, "Blue"),
    RED(255, 0, 0, "Red"),
    GREEN(0, 255, 0, "Green"),
    RAINBOW(0, 0, 0, "Rainbow");

    public int red;
    public int green;
    public int blue;
    public String name;

    private LEDState(int red, int green, int blue, String name) {
      this.red = red;
      this.green = green;
      this.blue = blue;
      this.name = name;
    }
  }
}
