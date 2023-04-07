// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.LED;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.LEDConstants;
import frc.robot.subsystems.LEDSubsystem;

/**
 * Command for settings the LEDs for a certain amount of time
 */
public class LEDTimerCommand extends CommandBase {
  private final LEDSubsystem m_subsystem;
  private final Timer m_timer = new Timer();

  private final int m_i, m_r, m_g, m_b;

  /** Creates a new LEDTimerCommand. */
  public LEDTimerCommand(LEDSubsystem subsystem, int i, int r, int g, int b) {
    m_subsystem = subsystem;
    addRequirements();

    m_i = i;
    m_r = r;
    m_g = g;
    m_b = b;
  }

  @Override
  public void initialize() {
    m_timer.restart();
    m_subsystem.setLED(m_i, m_r, m_g, m_b);
  }

  @Override
  public void end(boolean interrupted) {
    Color c = m_subsystem.getLED(m_i);
    if ((int) (c.red * 255) == m_r &&
        (int) (c.blue * 255) == m_b &&
        (int) (c.green * 255) == m_g) {
      m_subsystem.unsetLED(m_i);
    }
  }

  @Override
  public boolean isFinished() {
    return m_timer.hasElapsed(LEDConstants.kLEDOnTime);
  }
}
