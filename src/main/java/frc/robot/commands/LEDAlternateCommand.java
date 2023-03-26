// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.LEDConstants;
import frc.robot.subsystems.LEDSubsystem;

public class LEDAlternateCommand extends CommandBase {
  private final LEDSubsystem m_subsystem;

  private final int m_r1, m_r2, m_g1, m_g2, m_b1, m_b2;
  private final double m_time;

  private final Timer m_timer = new Timer();
  private int m_amount;
  private boolean cycle;

  /** Creates a new LEDAlternateCommand. */
  public LEDAlternateCommand(LEDSubsystem subsystem, int r1, int g1, int b1, int r2, int g2, int b2, double time,
      int amount) {
    m_subsystem = subsystem;
    addRequirements(m_subsystem);

    m_r1 = r1;
    m_r2 = r2;
    m_g1 = g1;
    m_g2 = g2;
    m_b1 = b1;
    m_b2 = b2;

    m_time = time;

    m_amount = amount;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_subsystem.setLED(0);
    cycle = true;
    m_timer.restart();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_timer.hasElapsed(m_time)) {
      if (m_amount == 0)
        cancel();

      else if (cycle && m_amount != 0)
        for (int i = 0; i < LEDConstants.kLEDLength; i++)
          m_subsystem.setLED(i, ((i % 2) == 0) ? m_r1 : m_r2, ((i % 2) == 0) ? m_g1 : m_g2,
              ((i % 2) == 0) ? m_b1 : m_b2);

      else if (m_amount != 0) {
        for (int i = 0; i < LEDConstants.kLEDLength; i++)
          m_subsystem.setLED(i, ((i % 2) == 1) ? m_r1 : m_r2, ((i % 2) == 1) ? m_g1 : m_g2,
              ((i % 2) == 1) ? m_b1 : m_b2);
      }

      cycle = !cycle;
      m_amount--;

      m_timer.restart();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    for (int i = 0; i < LEDConstants.kLEDLength; i++)
      m_subsystem.unsetLED(i);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_amount == 0;
  }
}
