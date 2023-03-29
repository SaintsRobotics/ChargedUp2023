// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.LEDConstants;
import frc.robot.subsystems.LEDSubsystem;

public class LEDCountdownCommand extends CommandBase {
  private final LEDSubsystem m_subsystem;
  private final Timer m_timer = new Timer();

  private int m_r;
  private int m_last;

  /**
   * Creates a new {@link LEDCountdownCommand}. Flashes the LEDs when less than 10
   * seconds are left in a match.
   * 
   * @param subsystem The required subsystem.
   */
  public LEDCountdownCommand(LEDSubsystem subsystem) {
    m_subsystem = subsystem;
    addRequirements(m_subsystem);
  }

  @Override
  public void initialize() {
    m_timer.restart();

    m_r = 100;
  }

  @Override
  public void execute() {
    int current = LEDConstants.kLEDLength - (int) (m_timer.get() * LEDConstants.kLEDLength);
    if (m_last != current) {
      for (var i = 0; i < current; i++) {
        m_subsystem.setLED(i, m_r, 0, 0);
      }
      m_subsystem.setLED(current, 0, 0, 0);
    }
    m_last = current;
    m_r -= 3;
    m_r = MathUtil.clamp(m_r, 0, 100);
    if (m_timer.hasElapsed(1)) {
      m_timer.reset();
      m_r = 100;
    }
  }

  @Override
  public void end(boolean interrupted) {
    m_subsystem.setLED(100, 0, 0);
  }

  @Override
  public InterruptionBehavior getInterruptionBehavior() {
    return InterruptionBehavior.kCancelIncoming;
  }
}
