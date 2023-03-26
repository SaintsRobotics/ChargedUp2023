// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LEDSubsystem;

public class LEDCommand extends CommandBase {
  private final LEDSubsystem m_LEDSubsystem;
  private final Timer m_timer = new Timer();
  private final BooleanSupplier m_isTipped;

  private boolean m_isRed;

  /**
   * Default command for LED subystem. Flashes red when tipped.
   * 
   * @param subsystem The required subsystem.
   * @param doubleSupplier  Supplier that returns true if the robot is tipped.
   */
  public LEDCommand(LEDSubsystem subsystem, BooleanSupplier doubleSupplier) {
    m_LEDSubsystem = subsystem;
    m_isTipped = doubleSupplier;
    addRequirements(m_LEDSubsystem);
  }

  @Override
  public void initialize() {
    m_timer.restart();
  }

  @Override
  public void execute() {
    if (m_timer.hasElapsed(0.3) && m_isTipped.getAsBoolean()) {
      m_LEDSubsystem.setLED(m_isRed ? 0 : 50, 0, 0);
      m_isRed = !m_isRed;
      m_timer.restart();
    }
  }
}
