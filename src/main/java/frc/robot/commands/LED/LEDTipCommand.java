// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.LED;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.LEDConstants;
import frc.robot.subsystems.LEDSubsystem;

/**
 * Command for making the LEDs flash red when the robot is tipping
 */
public class LEDTipCommand extends CommandBase {
  private final LEDSubsystem m_subsystem;
  private final AHRS m_gyro;
  private final Timer m_timer = new Timer();

  private double m_angle;
  private boolean isRed;

  /**
   * Creates a new {@link LEDTipCommand}. Flashes the LEDs based on the amount the
   * robot is tipped.
   * 
   * @param subsystem The required subsystem.
   * @param gyro      The gyro of the robot.
   */
  public LEDTipCommand(LEDSubsystem subsystem, AHRS gyro) {
    m_subsystem = subsystem;
    addRequirements(m_subsystem);

    m_gyro = gyro;
  }

  @Override
  public void initialize() {
    m_timer.restart();
    m_subsystem.setLED(100, 0, 0);
    isRed = true;
  }

  @Override
  public void execute() {
    m_angle = Math.max(Math.abs(m_gyro.getPitch()), Math.abs(m_gyro.getRoll()));
    if (m_angle > LEDConstants.kTipMax) {
      m_subsystem.setLED(100, 0, 0);
    } else if (m_timer.hasElapsed(0.3 - (m_angle / 185))) {
      m_subsystem.setLED(isRed ? 0 : 100, 0, 0);
      isRed = !isRed;
      m_timer.reset();
    }
  }

  @Override
  public void end(boolean interrupted) {
    m_subsystem.setLED(0, 0, 0);
  }

  @Override
  public boolean isFinished() {
    return m_angle < LEDConstants.kTipMin;
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }
}
