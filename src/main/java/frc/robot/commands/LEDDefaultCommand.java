// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.ArrayList;
import java.util.Random;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LEDSubsystem;

public class LEDDefaultCommand extends CommandBase {
  private final LEDSubsystem m_subsystem;
  private final ArrayList<Command> m_commands = new ArrayList<Command>();
  private Command m_command;

  /** Creates a new {@link LEDDefaultCommand}. */
  public LEDDefaultCommand(LEDSubsystem subsystem) {
    m_subsystem = subsystem;
    addRequirements(m_subsystem);

    m_commands.add(new LEDSwipeCommand(m_subsystem));
    m_commands.add(new LEDBlinkCommand(m_subsystem));
  }

  @Override
  public void initialize() {
    m_command = m_commands.get(new Random().nextInt(0, m_commands.size()));
    m_command.schedule();
  }

  @Override
  public void execute() {
    if (m_command.isFinished()) {
      m_command = m_commands.get(new Random().nextInt(0, m_commands.size()));
      m_command.schedule();
    }
  }

  @Override
  public void end(boolean interrupted) {
    m_command.cancel();
  }
}
