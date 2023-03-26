// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.LEDConstants;
import frc.robot.subsystems.LEDSubsystem;

public class LEDEffectCommand extends CommandBase {

  public enum EffectType {
    blink,
    swipeUp,
    midSplit,
    swipeDown
  }

  private EffectType m_type;
  private LEDSubsystem m_subsystem;
  private int m_r, m_g, m_b;
  private Timer m_timer = new Timer();
  private double m_time;

  private int m_internalState;
  private AddressableLEDBuffer m_buf = new AddressableLEDBuffer(LEDConstants.kLEDLength);
  private BooleanSupplier m_isTipped;

  /** Creates a new {@link LEDEffectCommand}. */
  public LEDEffectCommand(LEDSubsystem subsystem, EffectType type, int r, int g, int b, double time, BooleanSupplier isTipped) {
    m_subsystem = subsystem;
    m_type = type;
    m_r = r;
    m_g = g;
    m_b = b;
    m_time = time;
    m_isTipped = isTipped;
    addRequirements(m_subsystem);
  }

  @Override
  public void initialize() {
    for (int i = 0; i < LEDConstants.kLEDLength; i++)
      m_buf.setLED(i, m_subsystem.getState().getLED(i));

    m_internalState = m_type == EffectType.swipeDown ? LEDConstants.kLEDLength - 1 : 0;
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
          m_subsystem.setLED(m_r, m_g, m_b);
          m_internalState++;
        } else if (m_internalState == 1) {
          end(false);
          m_internalState++;
        } else if (m_internalState == 2)
          cancel();
        break;
      case swipeUp:
        if (m_internalState > 2 && m_internalState < LEDConstants.kLEDLength + 1)
          m_subsystem.setLED(m_internalState - 3,
              (int) (m_buf.getLED(m_internalState - 1).red * 255),
              (int) (m_buf.getLED(m_internalState - 1).green * 255),
              (int) (m_buf.getLED(m_internalState - 1).blue * 255));

        if (m_internalState == LEDConstants.kLEDLength + 2)
          cancel();
        else if (m_internalState < LEDConstants.kLEDLength) {
          m_subsystem.setLED(m_internalState, m_r, m_g, m_b);
        }

        m_internalState++;
        break;
      case midSplit:
        if (m_internalState > 2 && m_internalState < LEDConstants.kLEDLength + 1)
          m_subsystem.setLED(m_internalState - 3,
              (int) (m_buf.getLED(LEDConstants.kLEDLength - (m_internalState - 1) - 1).red * 255),
              (int) (m_buf.getLED(LEDConstants.kLEDLength - (m_internalState - 1) - 1).green * 255),
              (int) (m_buf.getLED(LEDConstants.kLEDLength - (m_internalState - 1) - 1).blue * 255));

        if (m_internalState == LEDConstants.kLEDLength + 2)
          cancel();
        else if (m_internalState < LEDConstants.kLEDLength) {
          m_subsystem.setLED(LEDConstants.kLEDLength - m_internalState - 1, m_r, m_g, m_b);
        }

        m_internalState++;
        break;
      case swipeDown:
        if (m_internalState < 25 && m_internalState > -4)
          m_subsystem.setLED(m_internalState + 3,
              (int) (m_buf.getLED(m_internalState + 3).red * 255),
              (int) (m_buf.getLED(m_internalState + 3).green * 255),
              (int) (m_buf.getLED(m_internalState + 3).blue * 255));

        if (m_internalState == -4) cancel();
        else if (m_internalState > -1) m_subsystem.setLED(m_internalState, m_r, m_g, m_b);
        m_internalState--;
    }
  }

  @Override
  public void end(boolean interrupted) {
    AddressableLEDBuffer buf = new AddressableLEDBuffer(LEDConstants.kLEDLength);
    for (int i = 0; i < LEDConstants.kLEDLength; i++)
      buf.setLED(i, m_buf.getLED(i));

    m_subsystem.setState(buf);
  }

  @Override
  public boolean isFinished() {
    return (m_type == EffectType.blink && m_internalState == 2) ||
        (m_type == EffectType.swipeUp && m_internalState == LEDConstants.kLEDLength + 2) ||
        (m_type == EffectType.midSplit && m_internalState == LEDConstants.kLEDLength + 2) ||
        (m_type == EffectType.swipeDown && m_internalState == -4) ||
        m_isTipped.getAsBoolean();
  }
}
