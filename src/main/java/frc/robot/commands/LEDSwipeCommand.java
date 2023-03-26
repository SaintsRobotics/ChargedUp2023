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
    kUp,
    kDown,
    kDouble,
    kSplit,
    kJoin,
    kDoubleUp,
    kDoubleDown,
    kLowDouble,
    kHighDouble,
    kDoubleDouble
  }

  private final LEDSubsystem m_subsystem;
  private final Type m_type;

  private final int m_r;
  private final int m_g;
  private final int m_b;

  private final boolean m_changeColor;

  private final Timer m_timer = new Timer();

  private int m_LEDIndex;

  /**
   * Creates a new {@link LEDSwipeCommand}.
   * 
   * @param subsystem   The required subsystem.
   * @param type        Type of swipe.
   * @param r           Red component (0-255)
   * @param g           Green component (0-255)
   * @param b           Blue component (0-255)
   * @param changeColor Whether the LEDs should end with the given color.
   */
  public LEDSwipeCommand(LEDSubsystem subsystem, Type type, int r, int g, int b, boolean changeColor) {
    m_subsystem = subsystem;
    addRequirements(m_subsystem);

    m_type = type;

    m_r = r;
    m_g = g;
    m_b = b;

    m_changeColor = changeColor;
  }

  @Override
  public void initialize() {
    m_timer.restart();
    m_LEDIndex = 0;
  }

  @Override
  public void execute() {
    int max = LEDConstants.kLEDLength - 1;
    int half = LEDConstants.kLEDLength / 2;
    if (m_timer.hasElapsed(LEDConstants.kSwipeTime)) {
      switch (m_type) {
        case kUp:
          setLED(m_LEDIndex, m_LEDIndex - LEDConstants.kSwipeOverlap);
          break;
        case kDown:
          setLED(max - m_LEDIndex, max - m_LEDIndex + LEDConstants.kSwipeOverlap);
          break;
        case kDouble:
          setLED(m_LEDIndex, m_LEDIndex - LEDConstants.kSwipeOverlap);
          setLED(max - m_LEDIndex, max - m_LEDIndex + LEDConstants.kSwipeOverlap);
          break;
        case kSplit:
          setLED((m_LEDIndex % half) + half, ((m_LEDIndex - LEDConstants.kSwipeOverlap) % half) + half);
          setLED((max - (m_LEDIndex % half)) - half, (max - ((m_LEDIndex - LEDConstants.kSwipeOverlap) % half)) - half);
          break;
        case kJoin:
          setLED(m_LEDIndex % half, (m_LEDIndex - LEDConstants.kSwipeOverlap) % half);
          setLED(max - (m_LEDIndex % half), max - ((m_LEDIndex - LEDConstants.kSwipeOverlap) % half));
          break;
        case kDoubleUp:
          setLED(m_LEDIndex % half, (m_LEDIndex - LEDConstants.kSwipeOverlap) % half);
          setLED((m_LEDIndex % half) + half, ((m_LEDIndex - LEDConstants.kSwipeOverlap) % half) + half);
          break;
        case kDoubleDown:
          setLED(max - (m_LEDIndex % half), max - ((m_LEDIndex - LEDConstants.kSwipeOverlap) % half));
          setLED((max - (m_LEDIndex % half)) - half, (max - ((m_LEDIndex - LEDConstants.kSwipeOverlap) % half) - half));
          break;
        case kLowDouble:
          setLED(m_LEDIndex % half, (m_LEDIndex - LEDConstants.kSwipeOverlap) % half);
          setLED((max - (m_LEDIndex % half)) - half, (max - ((m_LEDIndex - LEDConstants.kSwipeOverlap) % half)) - half);
          break;
        case kHighDouble:
          setLED((m_LEDIndex % half) + half, ((m_LEDIndex - LEDConstants.kSwipeOverlap) % half) + half);
          setLED(max - (m_LEDIndex % half), max - ((m_LEDIndex - LEDConstants.kSwipeOverlap) % half));
          break;
        case kDoubleDouble:
          setLED(m_LEDIndex % half, (m_LEDIndex - LEDConstants.kSwipeOverlap) % half);
          setLED((max - (m_LEDIndex % half)) - half, (max - ((m_LEDIndex - LEDConstants.kSwipeOverlap) % half)) - half);
          setLED((m_LEDIndex % half) + half, ((m_LEDIndex - LEDConstants.kSwipeOverlap) % half) + half);
          setLED(max - (m_LEDIndex % half), max - ((m_LEDIndex - LEDConstants.kSwipeOverlap) % half));
          break;
        default:
          break;
      }
      m_LEDIndex++;
      m_timer.reset();
    }
  }

  @Override
  public void end(boolean interrupted) {
    if (m_changeColor) {
      m_subsystem.setLED(m_r, m_g, m_b);
    }
  }

  @Override
  public boolean isFinished() {
    return m_LEDIndex == LEDConstants.kLEDLength + LEDConstants.kSwipeOverlap;
  }

  private void setLED(int setLEDIndex, int unsetLEDIndex) {
    if (m_LEDIndex < LEDConstants.kLEDLength) {
      m_subsystem.setLED(setLEDIndex, m_r, m_g, m_b);
    }
    if (m_LEDIndex >= LEDConstants.kSwipeOverlap && !m_changeColor) {
      m_subsystem.unsetLED(unsetLEDIndex);
    }
  }
}
