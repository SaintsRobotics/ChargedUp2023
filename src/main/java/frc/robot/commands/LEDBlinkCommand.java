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
 * Command for making the LEDs blink
 */
public class LEDBlinkCommand extends CommandBase {
  /**
   * Blink type
   */
  public enum BlinkType {
    /** Flashing blink */
    kBlink,
    /** Alternate blink */
    kAlternate;

    /** Random for generating random integers */
    private static final Random random = new Random();

    /** 
     * Generates a random BlinkType
     * @return The random BlinkType
     */
    public static BlinkType random() {
      BlinkType[] type = values();
      return type[random.nextInt(type.length)];
    }
  }

  /** LED subsystem */
  private final LEDSubsystem m_subsystem;

  /** Timer for controlling blink intervals */
  private final Timer m_timer = new Timer();

  /** Selected blink type */
  private BlinkType m_type;

  /** RGB values for blink color */
  private int m_r, m_g, m_b;

  /** Counter for number of iterations of blink */
  private int m_counter;

  /** Use random colors? */
  private final boolean m_random;

  /**
   * Creates a new {@link LEDBlinkCommand}.
   * 
   * @param subsystem The required subsystem.
   * @param type      Type of blink.
   * @param r         Red 0-255
   * @param g         Green 0-255
   * @param b         Blue 0-255
   */
  public LEDBlinkCommand(LEDSubsystem subsystem, BlinkType type, int r, int g, int b) {
    m_subsystem = subsystem;
    addRequirements(m_subsystem);

    m_type = type;

    m_r = r;
    m_g = g;
    m_b = b;

    m_random = false;
  }

  /**
   * Creates a new randomized {@link LEDBlinkCommand}.
   * 
   * @param subsystem The required subsystem.
   */
  public LEDBlinkCommand(LEDSubsystem subsystem) {
    m_subsystem = subsystem;
    addRequirements(m_subsystem);

    m_random = true;
  }

  @Override
  public void initialize() {
    // Generate random color
    if (m_random) {
      m_type = BlinkType.random();

      Random random = new Random();
      m_r = random.nextInt(0, 100);
      m_g = random.nextInt(0, 100);
      m_b = random.nextInt(0, 100);
    }

    m_timer.restart();
    m_counter = 0;
  }

  @Override
  public void execute() {
    // Keep track of current blink iteration
    if (m_timer.hasElapsed(LEDConstants.kBlinkTime)) {
      m_counter++;
      m_timer.restart();
    }

    // Use m_counter to determine the next blink state (on or off)
    boolean even = m_counter % 2 == 0;
    switch (m_type) {
      case kBlink:
        if (even) {
          m_subsystem.setLED(m_r, m_g, m_b, false);
        } else {
          m_subsystem.unsetLED();
        }
        break;
      case kAlternate:
        for (int i = 0; i < LEDConstants.kLEDLength; i++) {
          if (even) {
            m_subsystem.setLED(i, m_r, m_g, m_b);
          } else {
            m_subsystem.unsetLED(i);
          }
          even = !even;
        }
        break;
      default:
        break;
    }
  }

  @Override
  public void end(boolean interrupted) {
    // Restore LEDs to previous state
    m_subsystem.unsetLED();
  }

  @Override
  public boolean isFinished() {
    // Finished when we have cycled blink states enough times
    return m_counter >= LEDConstants.kBlinkAmount * 2;
  }

  @Override
  public boolean runsWhenDisabled() {
    // Allows command to run when disabled
    return true;
  }
}
