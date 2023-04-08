// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.LED;

import java.time.LocalTime;
import java.util.ArrayList;
import java.util.Random;
import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.LEDSubsystem;

/** 
 * Command for creating random LED effects
 */
public class LEDDefaultCommand extends CommandBase {
  private final LEDSubsystem m_subsystem;
  private final ArrayList<Command> m_commands = new ArrayList<Command>();
  private Command m_command;
  private final BooleanSupplier m_lockLED;

  /**
   * Creates a new {@link LEDDefaultCommand}.
   * 
   * @param subsystem The required subsystem.
   * @param lockLED Supplier to boolean which determines if we can set the LED. True if LEDs cannot be changed
   */
  public LEDDefaultCommand(LEDSubsystem subsystem, BooleanSupplier lockLED) {
    m_subsystem = subsystem;
    addRequirements(m_subsystem);

    Command foreverRainbowCommand = new LEDRainbowCommand(subsystem) {
      @Override
      public boolean isFinished() {
        return false;
      }
    };

    m_lockLED = lockLED;
    m_commands.add(new LEDSwipeCommand(m_subsystem));
    m_commands.add(new LEDBlinkCommand(m_subsystem));
    m_commands.add(new LEDSwipeCommand(m_subsystem)); //Add duplicate commands to increase their weight in the random draw
    m_commands.add(new LEDBlinkCommand(m_subsystem));
    m_commands.add(new LEDSwipeCommand(m_subsystem)); //Add duplicate commands to increase their weight in the random draw
    m_commands.add(new LEDBlinkCommand(m_subsystem));
    m_commands.add(new LEDSwipeCommand(m_subsystem)); //Add duplicate commands to increase their weight in the random draw
    m_commands.add(new LEDBlinkCommand(m_subsystem));
    m_commands.add(new ParallelDeadlineGroup(new WaitCommand(new Random(LocalTime.now().getSecond()).nextInt(1, 2)), foreverRainbowCommand));
  }

  @Override
  public void initialize() {
    if (!m_lockLED.getAsBoolean()) {
      m_command = m_commands.get(new Random(LocalTime.now().getSecond()).nextInt(0, m_commands.size()));
    }

    else {
      m_command = new CommandBase() {
        @Override
        public boolean isFinished() {
          return true;
        }
      };
    }

    m_command.schedule();
  }

  @Override
  public void execute() {
    if (m_command.isFinished() && !m_lockLED.getAsBoolean()) {
      m_command = m_commands.get(new Random(LocalTime.now().getSecond()).nextInt(0, m_commands.size()));
      m_command.schedule();
    }
  }

  @Override
  public void end(boolean interrupted) {
    m_command.cancel();
  }
}
