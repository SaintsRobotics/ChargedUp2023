// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants;

public class LEDSubsystem extends SubsystemBase {
  private final AddressableLED m_LED = new AddressableLED(LEDConstants.kLEDPort);
  private AddressableLEDBuffer m_LEDBuffer = new AddressableLEDBuffer(LEDConstants.kLEDLength);

  private int m_r;
  private int m_g;
  private int m_b;

  private final Timer m_timer = new Timer();
  private int i;

  /** Creates a new {@link LEDSubsystem}. */
  public LEDSubsystem() {
    m_LED.setLength(LEDConstants.kLEDLength);
    m_LED.setData(m_LEDBuffer);
    m_LED.start();
    m_timer.restart();
  }

  @Override
  public void periodic() {
    setLED(i, 0, 0, 100);

    if (m_timer.hasElapsed(0.15)) {
      i++;
      m_timer.reset();
    }
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
   * @param r Red 0-255
   * @param g Green 0-255
   * @param b Blue 0-255
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

  public void setLED(int r, int g, int b) {
    setLED(r, g, b, true);
  }

  /**
   * Resets individual LED to previous value.
   * 
   * @param i index of LED
   */
  public void unsetLED(int i) {
    setLED(i, m_r, m_g, m_b);
  }

  public void setLED(int i) {
    m_r = (int) (m_LEDBuffer.getLED(i).red * 255);
    m_b = (int) (m_LEDBuffer.getLED(i).blue * 255);
    m_g = (int) (m_LEDBuffer.getLED(i).green * 255);
  }

  /** Resets the LED to the default color (yellow) */
  public void setCone() {
    setLED(100, 100, 0);
  }

  public void setCube() {
    setLED(100, 0, 100);
  }
}
