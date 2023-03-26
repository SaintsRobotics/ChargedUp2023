// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants;

public class LEDSubsystem extends SubsystemBase {
  private final AddressableLED m_LED = new AddressableLED(LEDConstants.kLEDPort);
  private AddressableLEDBuffer m_LEDBuffer = new AddressableLEDBuffer(LEDConstants.kLEDLength);

  /** Creates a new {@link LEDSubsystem}. */
  public LEDSubsystem() {
    m_LED.setLength(LEDConstants.kLEDLength);
    m_LED.setData(m_LEDBuffer);
    m_LED.start();

    setCone();
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
    m_LEDBuffer.setRGB(i, r, g, b);
    m_LED.setData(m_LEDBuffer);
  }

  /**
   * Sets the rgb value for all LEDs.
   * 
   * @param r Red 0-255
   * @param g Green 0-255
   * @param b Blue 0-255
   */
  public void setLED(int r, int g, int b) {
    for (var i = 0; i < LEDConstants.kLEDLength; i++) {
      m_LEDBuffer.setRGB(i, r, g, b);
    }
    m_LED.setData(m_LEDBuffer);
  }

  /** Resets the LED to the default color (yellow) */
  public void setCone() {
    setLED(100, 100, 0);
  }

  public void setCube() {
    setLED(100, 0, 100);
  }

  public AddressableLEDBuffer getState(){
    return m_LEDBuffer;
  }

  public void setState(AddressableLEDBuffer buf){
    m_LEDBuffer = buf;
    m_LED.setData(m_LEDBuffer);
  }
}
