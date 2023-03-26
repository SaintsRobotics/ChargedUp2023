// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.LEDConstants;
import frc.robot.subsystems.LEDSubsystem;

public class LEDEffectCommand extends CommandBase {

  public enum EffectType {
    blink,
    swipe
  }

  private EffectType m_type;
  private LEDSubsystem m_subsystem;
  private Color m_col;
  private Timer m_timer = new Timer();
  private double m_time;

  private boolean m_isFinished;
  private int m_internalState;

  /** Creates a new {@link LEDEffectCommand}. */
  public LEDEffectCommand(LEDSubsystem subsystem, EffectType type, int r, int g, int b, double time) {
    m_subsystem = subsystem;
    m_type = type;
    m_col = new Color(r, g, b);
    m_time = time;
    addRequirements(m_subsystem);
  }

  @Override
  public void initialize() {
    m_isFinished = false;
    m_internalState = 0;
    evaluate();
    m_timer.restart();
  }

  @Override
  public void execute() {
    if (m_timer.hasElapsed(m_time)) {
      evaluate();
      m_timer.restart();
    }
  }

  public void evaluate() {
    switch (m_type) {
      case blink:
        if (m_internalState == 0) {
          m_subsystem.setLED((int) m_col.red, (int) m_col.green, (int) m_col.blue);
          m_internalState++;
        } else if (m_internalState == 1) {
          m_subsystem.setLED(0, 0, 0);
          m_internalState++;
        } else if (m_internalState == 2)
          m_isFinished = true;
        break;
      case swipe:
        if (m_internalState == LEDConstants.kLEDLength)
          m_isFinished = true;
        else {
          m_subsystem.setLED(m_internalState, (int) m_col.red, (int) m_col.green, (int) m_col.blue);
          m_internalState++;
        }
        break;
    }
  }

  @Override
  public void end(boolean interrupted) {
    m_subsystem.resetLED();
  }

  @Override
  public boolean isFinished() {
    return m_isFinished;
  }
}
