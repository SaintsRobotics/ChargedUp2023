// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Random;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.LEDConstants;
import frc.robot.subsystems.LEDSubsystem;

public class LEDSwipeCommand extends CommandBase {
  public enum SwipeType {
    kUp,
    kDown,
    kDouble,
    kSplit,
    kJoin,
    kDoubleUp,
    kDoubleDown,
    kLowDouble,
    kHighDouble,
    kDoubleDouble;

    private static final Random random = new Random();

    public static SwipeType random() {
      SwipeType[] type = values();
      return type[random.nextInt(type.length)];
    }
  }

  private final LEDSubsystem m_subsystem;
  private SwipeType m_type;

  private int m_r, m_g, m_b;

  private boolean m_changeColor;

  private final Timer m_timer = new Timer();

  private final boolean m_random;

  private int m_LEDIndex;

  /**
   * Creates a new {@link LEDSwipeCommand}.
   * 
   * @param subsystem   The required subsystem.
   * @param type        Type of swipe.
   * @param r           Red 0-255
   * @param g           Green 0-255
   * @param b           Blue 0-255
   * @param changeColor Whether the LEDs should end with the given color.
   */
  public LEDSwipeCommand(LEDSubsystem subsystem, SwipeType type, int r, int g, int b, boolean changeColor) {
    m_subsystem = subsystem;
    addRequirements(m_subsystem);

    m_type = type;

    m_r = r;
    m_g = g;
    m_b = b;

    m_changeColor = changeColor;

    m_random = false;
  }

  /**
   * Creates a new randomized {@link LEDSwipeCommand}.
   * 
   * @param subsystem The required subsystem.
   */
  public LEDSwipeCommand(LEDSubsystem subsystem) {
    m_subsystem = subsystem;
    addRequirements(m_subsystem);

    m_random = true;
  }

  @Override
  public void initialize() {
    if (m_random) {
      m_type = SwipeType.random();

      Random random = new Random();
      m_r = random.nextInt(0, 100);
      m_g = random.nextInt(0, 100);
      m_b = random.nextInt(0, 100);

      m_changeColor = random.nextBoolean();
    }
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
    if (m_changeColor && m_type != SwipeType.kHighDouble && m_type != SwipeType.kLowDouble) {
      m_subsystem.setLED(m_r, m_g, m_b);
    }
  }

  @Override
  public boolean isFinished() {
    if (m_changeColor) {
      switch (m_type) {
        case kDouble:
          return m_LEDIndex == LEDConstants.kLEDLength / 2 + LEDConstants.kSwipeOverlap;
        case kDoubleDouble:
          return m_LEDIndex == LEDConstants.kLEDLength / 4 + LEDConstants.kSwipeOverlap;
        case kDoubleDown:
          return m_LEDIndex == LEDConstants.kLEDLength / 2 + LEDConstants.kSwipeOverlap;
        case kDoubleUp:
          return m_LEDIndex == LEDConstants.kLEDLength / 2 + LEDConstants.kSwipeOverlap;
        case kDown:
          return m_LEDIndex == LEDConstants.kLEDLength + LEDConstants.kSwipeOverlap;
        case kHighDouble:
          return m_LEDIndex == LEDConstants.kLEDLength / 4 + LEDConstants.kSwipeOverlap;
        case kJoin:
          return m_LEDIndex == LEDConstants.kLEDLength / 2 + LEDConstants.kSwipeOverlap;
        case kLowDouble:
          return m_LEDIndex == LEDConstants.kLEDLength / 4 + LEDConstants.kSwipeOverlap;
        case kSplit:
          return m_LEDIndex == LEDConstants.kLEDLength / 2 + LEDConstants.kSwipeOverlap;
        case kUp:
          return m_LEDIndex == LEDConstants.kLEDLength + LEDConstants.kSwipeOverlap;
        default:
          return false;
      }
    }
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

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }
}
