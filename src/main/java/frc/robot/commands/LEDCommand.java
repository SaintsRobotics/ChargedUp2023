// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Queue;
import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.LEDEffectCommand.EffectType;
import frc.robot.subsystems.LEDSubsystem;

public class LEDCommand extends CommandBase {
  private final LEDSubsystem m_LEDSubsystem;
  private final Timer m_timer = new Timer();
  private final Timer m_effectTimer = new Timer();
  private final BooleanSupplier m_isTipped;

  private boolean m_isRed;
  private AddressableLEDBuffer m_buf;

  private Queue<SequentialCommandGroup> m_effectQueue;

  /**
   * Default command for LED subystem. Flashes red when tipped.
   * 
   * @param subsystem      The required subsystem.
   * @param booleanSupplier Supplier that returns true if the robot is tipped.
   */
  public LEDCommand(LEDSubsystem subsystem, BooleanSupplier booleanSupplier, Queue<SequentialCommandGroup> effectQueue) {
    m_LEDSubsystem = subsystem;
    m_isTipped = booleanSupplier;
    m_effectQueue = effectQueue;
    addRequirements(m_LEDSubsystem);
  }

  @Override
  public void initialize() {
    m_buf = m_LEDSubsystem.getState();
    m_timer.restart();
    m_effectTimer.restart();
  }

  @Override
  public void execute() {
    if (m_timer.hasElapsed(0.15) && m_isTipped.getAsBoolean()) {
      m_LEDSubsystem.setLED(m_isRed ? 0 : 100, 0, 0);
      m_isRed = !m_isRed;
      m_timer.restart();
    }

    if (m_effectTimer.hasElapsed(10)) {
      SequentialCommandGroup cmd = new SequentialCommandGroup(
          new LEDEffectCommand(m_LEDSubsystem, EffectType.blink, 0, 0, 100, 0.1, m_isTipped),
          new WaitCommand(0.1),
          new LEDEffectCommand(m_LEDSubsystem, EffectType.blink, 0, 0, 100, 0.1, m_isTipped),
          new LEDEffectCommand(m_LEDSubsystem, EffectType.swipeUp, 0, 0, 100, 0.02, m_isTipped),
          new LEDEffectCommand(m_LEDSubsystem, EffectType.midSplit, 0, 0, 100, 0.02, m_isTipped),
          new LEDEffectCommand(m_LEDSubsystem, EffectType.swipeDown, 0, 0, 100, 0.02, m_isTipped),
          new LEDEffectCommand(m_LEDSubsystem, EffectType.blink, 0, 0, 100, 0.1, m_isTipped),
          new WaitCommand(0.1),
          new LEDEffectCommand(m_LEDSubsystem, EffectType.blink, 0, 0, 100, 0.1, m_isTipped));
      
      cmd.schedule();
      m_effectQueue.add(cmd);
      m_effectTimer.reset();
    }
  }

  @Override
  public void end(boolean interrupted) {
    m_LEDSubsystem.setState(m_buf);
  }
}
