// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Queue;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.LEDBlinkCommand.BlinkType;
import frc.robot.subsystems.LEDSubsystem;

public class LEDCommand extends CommandBase {
  private final LEDSubsystem m_LEDSubsystem;
  private final Timer m_timer = new Timer();
  private final Timer m_effectTimer = new Timer();

  private Queue<SequentialCommandGroup> m_effectQueue;

  /**
   * Default command for LED subystem. Flashes red when tipped.
   * 
   * @param subsystem       The required subsystem.
   * @param booleanSupplier Supplier that returns true if the robot is tipped.
   */
  public LEDCommand(LEDSubsystem subsystem, Queue<SequentialCommandGroup> effectQueue) {
    m_LEDSubsystem = subsystem;
    m_effectQueue = effectQueue;
    addRequirements(m_LEDSubsystem);
  }

  @Override
  public void initialize() {
    m_timer.restart();
    m_effectTimer.restart();
  }

  @Override
  public void execute() {

    if (m_effectTimer.hasElapsed(5)) {
      SequentialCommandGroup cmd = new SequentialCommandGroup(
          new LEDBlinkCommand(m_LEDSubsystem, BlinkType.kAlternate, 0, 0, 100),
          new WaitCommand(0.3),
          new LEDBlinkCommand(m_LEDSubsystem, BlinkType.kAlternate, 100, 0, 0));

      cmd.schedule();
      m_effectQueue.add(cmd);
      m_effectTimer.reset();
    }
  }
}
