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

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  private Timer m_buttonTimer = new Timer();

  private boolean m_wasClicked = false, m_clickAction = false;

  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();
    m_buttonTimer.restart();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {
    m_clickAction = m_wasClicked = false;
  }

  @Override
  public void disabledPeriodic() {
    // Quickly press and release user button to toggle compressor
    // Press and hold for 5 seconds to set pivot to coastm mode
    //  Release to set pivot to break mode

    // Check if button is currently pressed
    if (RobotController.getUserButton()) {
      if (!m_wasClicked) { //Check if button was not already clicked
        m_wasClicked = m_clickAction = true; // Store that button was clicked and allow button action
        m_buttonTimer.restart(); //Start the timer
      }
      else if (m_clickAction && m_buttonTimer.get() > 5) { // Check if enough time has passed and there is a click action
        m_clickAction = false; // Remove click action
        m_robotContainer.setPivotMode(IdleMode.kCoast); // Set pivot mode to coast
      }
    }

    // Check if button is not clicked
    else {
      if (m_clickAction) m_robotContainer.grabberSubsystem.toggleCompressor(); // Only toggle compressor if there is still a click action
      else m_robotContainer.setPivotMode(IdleMode.kBrake); // If click action was used, pivot is in coast mode, so set it back to break mode
      m_wasClicked = m_clickAction = false; // Remove click action and store that button was not clicked
    }
  }

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
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
