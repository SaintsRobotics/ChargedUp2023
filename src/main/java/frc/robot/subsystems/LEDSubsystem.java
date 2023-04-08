// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.time.LocalTime;
import java.util.Random;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants;

public class LEDSubsystem extends SubsystemBase {
  private final AddressableLED m_LED = new AddressableLED(LEDConstants.kLEDPort);
  private final AddressableLEDBuffer m_LEDBuffer = new AddressableLEDBuffer(LEDConstants.kLEDLength);

  private int m_r, m_g, m_b;
  
  private static final int[][] m_colors = new int[][] {{100, 0, 0}, {0, 100, 0}, {0, 0, 100}, {100, 100, 0}, {0, 100, 100}, {100, 0, 100}};

  /** Creates a new {@link LEDSubsystem}. */
  public LEDSubsystem() {
    m_LED.setLength(LEDConstants.kLEDLength);
    m_LED.setData(m_LEDBuffer);
    m_LED.start();
  }

  /**
   * Sets individual LED
   * 
   * @param i index of LED
   * @param r Red 0-255
   * @param g Green 0-255
   * @param b Blue 0-255
   */
  public void setLED(int i, int r, int g, int b) {
    if (i >= 0 && i < LEDConstants.kLEDLength) {
      m_LEDBuffer.setRGB(i, r, g, b);
      m_LED.setData(m_LEDBuffer);
    }
  }

  /**
   * Sets the rgb value for all LEDs.
   * 
   * @param r     Red 0-255
   * @param g     Green 0-255
   * @param b     Blue 0-255
   * @param store Whether to save the color.
   */
  public void setLED(int r, int g, int b, boolean store) {
    for (var i = 0; i < LEDConstants.kLEDLength; i++) {
      m_LEDBuffer.setRGB(i, r, g, b);
    }
    m_LED.setData(m_LEDBuffer);

    if (store) {
      m_r = r;
      m_g = g;
      m_b = b;
    }
  }

  /**
   * Sets the rgb value for all LEDs and stores the color.
   * 
   * @param r Red 0-255
   * @param g Green 0-255
   * @param b Blue 0-255
   */
  public void setLED(int r, int g, int b) {
    setLED(r, g, b, true);
  }

  /**
   * Sets the HSV value for all LEDs.
   * 
   * @param i Index of LED.
   * @param h the h value [0-180)
   * @param s the s value [0-255]
   * @param v the v value [0-255]
   */
  public void setHSV(int i, int h, int s, int v) {
    m_LEDBuffer.setHSV(i, h, s, v);
    m_LED.setData(m_LEDBuffer);
  }

  /**
   * Resets individual LED to previous value.
   * 
   * @param i index of LED
   */
  public void unsetLED(int i) {
    setLED(i, m_r, m_g, m_b);
  }

  /**
   * Resets all LEDs to previous value.
   */
  public void unsetLED() {
    setLED(m_r, m_g, m_b);
  }

  /**
   * Returns the color of the LED.
   * 
   * @param i Index of LED.
   * @return Color of LED.
   */
  public Color getLED(int i) {
    return m_LEDBuffer.getLED(i);
  }

  /** Resets the LED to the default color (yellow) */
  public void setCone() {
    setLED(100, 100, 0);
  }

  public void setCube() {
    setLED(100, 0, 100);
  }

  public static int[] generateRandomColor() {
    return m_colors[new Random(LocalTime.now().getSecond()).nextInt(0, m_colors.length)];
  }
}
