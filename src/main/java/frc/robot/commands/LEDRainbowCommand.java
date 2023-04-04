// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.LEDConstants;
import frc.robot.subsystems.LEDSubsystem;

public class LEDRainbowCommand extends CommandBase {
  private final LEDSubsystem m_subsystem;
  private final Timer m_timer = new Timer();
  private int m_hue;

  /**
   * Creates a new {@link LEDRainbowCommand}.
   * 
   * @param subsystem The required subsystem.
   */
  public LEDRainbowCommand(LEDSubsystem subsystem) {
    m_subsystem = subsystem;
    addRequirements(m_subsystem);
  }

  @Override
  public void initialize() {
    m_timer.restart();
    m_hue = 0;
  }

  @Override
  public void execute() {
    if (m_timer.hasElapsed(0.02)) {
      for (var i = 0; i < LEDConstants.kLEDLength; i++) {
        m_subsystem.setHSV(i, m_hue + 2 * i, 255, 100);
      }
      m_hue += 2;
      m_timer.reset();
    }
  }

  @Override
  public void end(boolean interrupted) {
    m_subsystem.unsetLED();
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }
}
