// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.LED.LEDBlinkCommand;
import frc.robot.commands.LED.LEDBlinkCommand.BlinkType;

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

  private boolean m_wasClicked = false, m_clickAction = false;

  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();

    m_idleCommand = m_robotContainer.getIdleCommand();
    m_startupCommand = m_robotContainer.getStartupCommand();

    if (m_startupCommand != null) {
      m_startupCommand.schedule();
    }

    m_buttonTimer.restart();
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
    m_clickAction = m_wasClicked = false;
  }

  @Override
  public void disabledPeriodic() {
    // Quickly press and release user button to toggle compressor
    // Press and hold user button for 3 seconds to set pivot to coastm mode
    // Release user button to set pivot to break mode

    // Check if button is currently pressed
    if (RobotController.getUserButton()) {
      if (!m_wasClicked) { // Check if button was not already clicked
        m_wasClicked = m_clickAction = true; // Store that button was clicked and allow button action
        m_buttonTimer.restart(); // Start the timer
      } else if (m_clickAction && m_buttonTimer.get() > 3) { // Check if enough time has passed and there is a click
                                                             // action
        m_clickAction = false; // Remove click action
        m_robotContainer.setPivotMode(IdleMode.kCoast); // Set pivot mode to coast

        if (m_idleCommand != null)
          m_idleCommand.cancel();
        if (m_startupCommand != null)
          m_startupCommand.cancel();

        new LEDBlinkCommand(m_robotContainer.LEDSubsystem, BlinkType.kBlink, 100, 0, 0).schedule();;
        m_robotContainer.setLEDCriticalRed(false);
      }
    }

    // Check if button is not clicked
    else {
      if (m_clickAction) {
        if (m_idleCommand != null)
          m_idleCommand.cancel();
        m_robotContainer.grabberSubsystem.toggleCompressor(); // Only toggle compressor if there is still a click
        if (m_idleCommand != null)
          m_idleCommand.cancel();
        m_idleCommand = m_robotContainer.getIdleCommand();
        new LEDBlinkCommand(m_robotContainer.LEDSubsystem, BlinkType.kBlink, 100, 50, 0).andThen(m_idleCommand).schedule();
      }

      else if (m_wasClicked) {
        m_robotContainer.setPivotMode(IdleMode.kBrake); // If click action was used, pivot is in coast mode, so set it
                                                        // back to break mode
        m_robotContainer.setLEDCriticalRed(true);
        if (m_idleCommand != null)
          m_idleCommand.cancel();
        m_idleCommand = m_robotContainer.getIdleCommand();
        new WaitCommand(2).andThen(m_idleCommand).schedule();
      }
      m_wasClicked = m_clickAction = false; // Remove click action and store that button was not clicked

      if (m_idleCommand != null && m_startupCommand != null && m_idleCommand.isFinished()
          && m_startupCommand.isFinished())
        m_idleCommand.schedule();
    }
  }

  @Override
  public void autonomousInit() {
    m_robotContainer.setPivotMode(IdleMode.kBrake);

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
    m_robotContainer.setPivotMode(IdleMode.kBrake);
    
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
