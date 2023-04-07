// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private Command m_startupCommand;
  private Command m_idleCommand;

  private RobotContainer m_robotContainer;

  private Timer m_buttonTimer = new Timer();

  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();
    m_buttonTimer.start();

    m_startupCommand = m_robotContainer.getStartupCommand();
    m_idleCommand = m_robotContainer.getIdleCommand();

    if (m_startupCommand != null) {
      m_startupCommand.schedule();
    }
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {
    if (!m_startupCommand.isScheduled() && m_idleCommand != null) {
      m_idleCommand.schedule();
    }
  }

  @Override
  public void disabledPeriodic() {
    // If User button on the RoboRIO is pressed while robot is disabled, then do not
    // start the compressor
    if (RobotController.getUserButton() && m_buttonTimer.get() > 1) {
      m_robotContainer.grabberSubsystem.toggleCompressor();
      m_buttonTimer.reset();
    }
  }

  @Override
  public void autonomousInit() {
    m_robotContainer.unlockLED();
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }

    if (m_startupCommand != null) {
      m_startupCommand.cancel();
    }

    if (m_idleCommand != null) {
      m_idleCommand.cancel();
    }
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    m_robotContainer.unlockLED();
    
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    if (m_startupCommand != null) {
      m_startupCommand.cancel();
    }

    if (m_idleCommand != null) {
      m_idleCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {
  }
}
