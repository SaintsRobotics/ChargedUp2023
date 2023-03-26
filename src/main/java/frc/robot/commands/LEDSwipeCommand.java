// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.LEDConstants;
import frc.robot.subsystems.LEDSubsystem;

public class LEDSwipeCommand extends CommandBase {
  public enum Type {
    kDown,
    kUp,
    kDouble
  }

  private final LEDSubsystem m_subsystem;
  private final Type m_type;

  private final int m_r;
  private final int m_g;
  private final int m_b;

  private final Timer m_timer = new Timer();

  private int m_LEDIndex;

  /**
   * Creates a new {@link LEDSwipeCommand}.
   * 
   * @param subsystem The required subsystem.
   * @param type      Type of swipe.
   * @param r         Red component (0-255)
   * @param g         Green component (0-255)
   * @param b         Blue component (0-255)
   */
  public LEDSwipeCommand(LEDSubsystem subsystem, Type type, int r, int g, int b) {
    m_subsystem = subsystem;
    addRequirements(m_subsystem);

    m_type = type;

    m_r = r;
    m_g = g;
    m_b = b;
  }

  @Override
  public void initialize() {
    m_timer.restart();
    m_LEDIndex = 0;
  }

  @Override
  public void execute() {
    if (m_timer.hasElapsed(LEDConstants.kSwipeTime)) {
      switch (m_type) {
        case kDown:
          m_subsystem.setLED(m_LEDIndex, m_r, m_g, m_b);
          m_subsystem.unsetLED(
              m_LEDIndex - LEDConstants.kSwipeOverlap);
          break;
        case kUp:
          m_subsystem.setLED(LEDConstants.kLEDLength - 1 - m_LEDIndex, m_r, m_g, m_b);
          m_subsystem.unsetLED(
              LEDConstants.kLEDLength - 1 - m_LEDIndex + LEDConstants.kSwipeOverlap);
          break;
        case kDouble:
          m_subsystem.setLED(m_LEDIndex, m_r, m_g, m_b);
          m_subsystem.unsetLED(
              m_LEDIndex - LEDConstants.kSwipeOverlap);
          m_subsystem.setLED(LEDConstants.kLEDLength - 1 - m_LEDIndex, m_r, m_g, m_b);
          m_subsystem.unsetLED(
              LEDConstants.kLEDLength - 1 - m_LEDIndex + LEDConstants.kSwipeOverlap);
          break;
      }
      m_LEDIndex++;
      m_timer.reset();
    }
  }

  @Override
  public boolean isFinished() {
    return m_LEDIndex == LEDConstants.kLEDLength + LEDConstants.kSwipeOverlap;
  }
}
