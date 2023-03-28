// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LEDSubsystem;

/** Uses a PID and the gyroscope to balance the robot on the charger. */
public class BalanceCommand extends CommandBase {
  private final DriveSubsystem m_driveSubsystem;
  private final LEDSubsystem m_LEDSubsystem;

  private final PIDController m_PID = new PIDController(0.025, 0, 0);
  private final Timer m_timer = new Timer();

  private boolean m_isRed;
  private int fade = 255;

  /**
   * Creates a new {@link BalanceCommand}.
   * 
   * @param driveSubsystem The required drive subsystem.
   * @param LEDSubsystem   The required LED subsystem.
   */
  public BalanceCommand(DriveSubsystem driveSubsystem, LEDSubsystem LEDSubsystem) {
    m_driveSubsystem = driveSubsystem;
    m_LEDSubsystem = LEDSubsystem;
    addRequirements(m_driveSubsystem, m_LEDSubsystem);

    m_PID.setTolerance(DriveConstants.kToleranceBalance);
  }

  @Override
  public void initialize() {
    m_timer.restart();
    fade = 100;
  }

  @Override
  public void execute() {
    m_driveSubsystem.drive(
        m_PID.calculate(m_driveSubsystem.getGyroPitch(), 0),
        0,
        0,
        false);

    if (m_timer.hasElapsed(0.1) && m_PID.atSetpoint()) {
      m_LEDSubsystem.setLED(0, fade, 0, false);
      fade = (fade - 10) % 51;
      m_timer.reset();
    } else if (m_timer.hasElapsed(0.3)) {
      fade = 100;
      m_LEDSubsystem.setLED(m_isRed ? 0 : 100, 0, 0, false);
      m_isRed = !m_isRed;
      m_timer.reset();
    }
  }

  @Override
  public void end(boolean interrupted) {
    m_driveSubsystem.drive(0, 0, 0, false);
    m_LEDSubsystem.unsetLED();
  }
}
