// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Random;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.LEDConstants;
import frc.robot.subsystems.LEDSubsystem;

/**
 * Command for making LEDs swipe
 */
public class LEDSwipeCommand extends CommandBase {
  /**
   * How the LEDs should swipe
   */
  public enum SwipeType {
    /** Bottom to top */
    kUp,

    /** Top to bottom */
    kDown,

    /** kUp and kDown at the same time */
    kDouble,

    /** Start from the center and split around */
    kSplit,

    /** kSplit backwards */
    kJoin,

    /** Two kUps starting at different positions at the same time */
    kDoubleUp,

    /** Two kDowns starting at different positions at the same time */
    kDoubleDown,

    /** kDoubleDown on the low end of the LED strip */
    kLowDouble,

    /** kDoubleUp on the high end of the LED strip */
    kHighDouble,

    /** kLowDouble and kHighDouble at the same time */
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
          setLED(m_LEDIndex);
          break;
        case kDown:
          setLED(max - m_LEDIndex);
          break;
        case kDouble:
          setLED(m_LEDIndex);
          setLED(max - m_LEDIndex);
          break;
        case kSplit:
          setLED((m_LEDIndex % half) + half);
          setLED((max - (m_LEDIndex % half)) - half);
          break;
        case kJoin:
          setLED(m_LEDIndex % half);
          setLED(max - (m_LEDIndex % half));
          break;
        case kDoubleUp:
          setLED(m_LEDIndex % half);
          setLED((m_LEDIndex % half) + half);
          break;
        case kDoubleDown:
          setLED(max - (m_LEDIndex % half));
          setLED((max - (m_LEDIndex % half)) - half);
          break;
        case kLowDouble:
          setLED(m_LEDIndex % half);
          setLED((max - (m_LEDIndex % half)) - half);
          break;
        case kHighDouble:
          setLED((m_LEDIndex % half) + half);
          setLED(max - (m_LEDIndex % half));
          break;
        case kDoubleDouble:
          setLED(m_LEDIndex % half);
          setLED((max - (m_LEDIndex % half)) - half);
          setLED((m_LEDIndex % half) + half);
          setLED(max - (m_LEDIndex % half));
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
          return m_LEDIndex == LEDConstants.kLEDLength / 2;
        case kDoubleDouble:
          return m_LEDIndex == LEDConstants.kLEDLength / 4;
        case kDoubleDown:
          return m_LEDIndex == LEDConstants.kLEDLength / 2;
        case kDoubleUp:
          return m_LEDIndex == LEDConstants.kLEDLength / 2;
        case kDown:
          return m_LEDIndex == LEDConstants.kLEDLength;
        case kHighDouble:
          return m_LEDIndex == LEDConstants.kLEDLength / 4;
        case kJoin:
          return m_LEDIndex == LEDConstants.kLEDLength / 2;
        case kLowDouble:
          return m_LEDIndex == LEDConstants.kLEDLength / 4;
        case kSplit:
          return m_LEDIndex == LEDConstants.kLEDLength / 2;
        case kUp:
          return m_LEDIndex == LEDConstants.kLEDLength;
        default:
          return false;
      }
    }
    return m_LEDIndex == LEDConstants.kLEDLength;
  }

  private void setLED(int i) {
    if (m_changeColor) {
      m_subsystem.setLED(i, m_r, m_g, m_b);
    } else {
      new LEDTimerCommand(m_subsystem, i, m_r, m_g, m_b).schedule();
    }
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }
}
